/* ESP32-S3 burst SDR RX + experimental raw-I/Q TX over native USB Serial/JTAG and UART0. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"

#include "burst_serial.h"
#include "rx_tuning.h"
#include "display.h"

SOC_RESERVE_MEMORY_REGION(0x3fcd0000, 0x3fce0000, s3_rf_dump);
#define IQ_WORDS 16380u
#define IQ_BUFFER ((uint32_t *)0x3fcd0000)
#define SRAM_OWNER_REG 0x600c101cu
#define S3_TX_CTRL_REG 0x60033d64u
#define S3_RX_CTRL_REG 0x60033d5cu

extern void adctrig(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
/* The REAL vendor DAC transmit from librftest.a. Uses ctrl reg 0x60033d64,
 * fills 0x3fcd0000 with a ramp, and calls two more librftest funcs at the end
 * (the DAC/PA enable the project's RE omitted). Signature: (words,clock,flag,arg). */
extern void dactrig(uint32_t,uint32_t,uint32_t,uint32_t);
extern void stop_tx_tone(unsigned);
#define phy_stop_tx_tone stop_tx_tone
extern void force_txon_mode(unsigned,unsigned,unsigned);
extern void rom_pbus_xpd_rx_off(void);   /* 0x40005e38 */
extern void rom_pbus_xpd_tx_on(unsigned);/* 0x40005e5c - enable TX mixer/PA */
extern void rom_dac_rate_set(unsigned);  /* 0x400061ec - enable/set DAC clock */
extern void rom_tx_pwctrl_bg_init(void); /* 0x40005f64 - TX power control init */
extern void rom_pbus_workmode(void);
#define phy_pbus_workmode rom_pbus_workmode
extern void rom_pbus_xpd_rx_on(unsigned);
#define phy_pbus_xpd_rx_on rom_pbus_xpd_rx_on
extern void rom_pbus_xpd_tx_off(void);
#define phy_pbus_xpd_tx_off rom_pbus_xpd_tx_off
extern void rom_set_rxclk_en(unsigned);
#define phy_set_rxclk_en rom_set_rxclk_en
extern void set_chanfreq(unsigned,unsigned);
extern void set_rf_freq_offset(unsigned,unsigned,int);

static void s3_tune(unsigned mhz) {
    bool channel=(mhz>=2412 && mhz<=2472 && (mhz-2412)%5==0)||mhz==2484;
    set_chanfreq(channel?mhz:2412,0);
    if(!channel)set_rf_freq_offset(0,mhz,0);
}

#define S3_FREQ_MIN RX_FREQ_MIN
#define S3_FREQ_MAX RX_FREQ_MAX
static unsigned frequency_mhz=2412;
static bool rx_ready;
enum { rx_source=0,rx_mode=0,rx_flag=0,rx_wide=0,rx_prep=3,rx_pack=0,rx_agc=0 };

extern void force_rx_gain(unsigned,unsigned,unsigned);
static int rx_filter=-1;
extern unsigned rom_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void rom_chip_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
static unsigned rx_filter_saved[2];

static void rx_filter_apply(void) {
    for(unsigned j=0;j<2;j++) {
        rx_filter_saved[j]=rom_chip_i2c_readReg(0x67,0,4+j);
        if(rx_filter>=0)
            rom_chip_i2c_writeReg(0x67,0,4+j,(rx_filter_saved[j]&~63u)|(unsigned)rx_filter);
    }
}
static void rx_filter_restore(void) {
    if(rx_filter>=0)
        for(unsigned j=0;j<2;j++)
            rom_chip_i2c_writeReg(0x67,0,4+j,rx_filter_saved[j]);
}

int cmd_parse(char *cmd,char *name,int *argc,char **argv) {
    (void)cmd;(void)name;(void)argc;(void)argv;return -1;
}

#define send_bytes burst_serial_send
static void reply(const char *s) { (void)send_bytes(s,strlen(s)); }
#include "burst_gain.h"
#include "burst_limits.h"

static void prepare_rx(void) {
    if(rx_ready)return;
    s3_tune(frequency_mhz);
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    gain_apply();
    rx_ready=true;
}

#include "filter_probe.h"

static size_t packed_size(unsigned n) { return (n*20u+7u)/8u; }

static void pack_iq(unsigned n) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<n;j+=2,p+=5) {
        uint32_t a=IQ_BUFFER[j]&0xfffffu;
        uint32_t b=j+1<n?IQ_BUFFER[j+1]&0xfffffu:0;
        p[0]=a;p[1]=a>>8;p[2]=(a>>16)|(b<<4);
        if(j+1<n){p[3]=b>>4;p[4]=b>>12;}
    }
}

static void pack_iq8(unsigned n) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<n;j++) {
        uint32_t w=IQ_BUFFER[j];
        p[2*j]=(w>>2)&255;
        p[2*j+1]=(w>>12)&255;
    }
}

static size_t wire_size(unsigned n,unsigned format) {
    return format==16?n*2:format==20?packed_size(n):n*4;
}

static bool capture(unsigned n,unsigned divider,unsigned format) {
    if(divider!=0 && divider!=1 && divider!=6){reply("ERR rate\n");return false;}
    prepare_rx();

    for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
    rx_filter_apply();
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    int64_t start=esp_timer_get_time();
    REG_WRITE(0x60033d5c,0);
    REG_WRITE(0x60033d90,rx_pack|((rx_pack+1)<<6)|((rx_pack+2)<<12)|((rx_pack+3)<<18)|(rx_agc<<24));
    REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    uint32_t rate_bits=divider==1?(1u<<15):divider==6?(1u<<16):0;
    uint32_t ctrl=0x80000000u|rate_bits|(rx_wide<<17)|(rx_flag<<16)|(rx_source<<20)|(rx_mode<<28)|n;
    REG_WRITE(0x60033d5c,ctrl);
    REG_WRITE(0x60033d5c,ctrl|(1u<<19));
    REG_WRITE(0x60033d5c,ctrl);
    while(!(REG_READ(0x60033d5c)&(1u<<18)) && esp_timer_get_time()-start<20000){}
    bool done=(REG_READ(0x60033d5c)&(1u<<18))!=0;
    uint32_t elapsed=esp_timer_get_time()-start;
    REG_WRITE(0x60033d5c,0);
    REG_WRITE(SRAM_OWNER_REG,owner);
    rx_filter_restore();
    if(!done){reply("ERR capture_timeout\n");return false;}
    for(unsigned j=0;j<n;j++) {
        if(IQ_BUFFER[j]==0xa5a0055au){reply("ERR capture_timeout\n");return false;}
    }
    size_t bytes=wire_size(n,format);
    if(format==16)pack_iq8(n);else if(format==20)pack_iq(n);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,bytes);
    char h[96];
    snprintf(h,sizeof(h),"DATA %u %08" PRIx32 " %" PRIu32 "\n",n,crc,elapsed);
    bool sent=send_bytes(h,strlen(h)) && send_bytes(IQ_BUFFER,bytes);
    if(sent && format==16) {
        char lab[24];
        snprintf(lab,sizeof(lab),"%u MHz",frequency_mhz);
        display_waveform((const int8_t *)IQ_BUFFER,n,lab,false);
    }
    return sent;
}

static unsigned s_dac_rate=0;
static void prepare_tx(unsigned dac_rate);
static void tx_teardown(void);

/* Reverse engineered from the pinned ESP32-S3 librftest.a::dactrig().
 * dactrig() normally overwrites the SRAM with a vendor ramp; this function
 * programs only the replay half so host-provided words survive. */
static bool tx_dac_replay(unsigned words, unsigned clock_arg, unsigned flag_arg) {
    if(!words || words>IQ_WORDS || clock_arg>1 || flag_arg>1) {
        reply("ERR tx_settings\n");return false;
    }

    /* Put the PHY into the same TX state wifiscwout uses (keys the PA on-channel),
     * then run the real-dactrig register sequence on 0x60033d64 with OUR data. */
    prepare_tx(s_dac_rate);

    uint32_t ctrl=REG_READ(S3_TX_CTRL_REG)&0x7fffffffu;
    REG_WRITE(S3_TX_CTRL_REG,ctrl);

    ctrl=REG_READ(S3_TX_CTRL_REG);
    ctrl=(ctrl&0xffffc000u)|(words&0x3fffu);
    if(clock_arg==1)ctrl&=~BIT(15);else ctrl|=BIT(15);
    ctrl&=~BIT(19);
    ctrl=(ctrl&0xf00fffffu)|((words&0xffu)<<20);
    REG_WRITE(S3_TX_CTRL_REG,ctrl);

    /* flag_arg now selects a sustained transmission: one replay pass is only
     * ~200us, far too short for a separate RX board to catch, so we re-trigger
     * each completed pass for a fixed on-air window. flag_arg=0 keeps the
     * original single-shot behaviour. */
    int64_t start=esp_timer_get_time();
    REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
    bool done;
    if(flag_arg) {
        while(esp_timer_get_time()-start < 150000) {   /* 150 ms sustained */
            if(REG_READ(S3_TX_CTRL_REG)&BIT(18)) {
                REG_WRITE(S3_TX_CTRL_REG,ctrl);
                REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
            }
        }
        done=true;
    } else {
        while(!(REG_READ(S3_TX_CTRL_REG)&BIT(18)) &&
              esp_timer_get_time()-start<20000) {}
        done=(REG_READ(S3_TX_CTRL_REG)&BIT(18))!=0;
    }
    uint32_t elapsed=(uint32_t)(esp_timer_get_time()-start);

    REG_CLR_BIT(S3_TX_CTRL_REG,BIT(31));
    REG_WRITE(S3_RX_CTRL_REG,0);
    tx_teardown();
    rx_ready=false;
    prepare_rx();

    if(!done){reply("ERR tx_timeout\n");return false;}
    char h[80];
    snprintf(h,sizeof(h),"TXDONE %u %" PRIu32 "\n",words,elapsed);
    reply(h);
    return true;
}

static bool tx_receive_payload(size_t bytes, uint32_t expected_crc) {
    char h[64];
    snprintf(h,sizeof(h),"READY %u\n",(unsigned)bytes);
    reply(h);
    if(!burst_serial_receive(IQ_BUFFER,bytes,5000)) {
        reply("ERR tx_upload_timeout\n");return false;
    }
    uint32_t actual=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,bytes);
    if(expected_crc && actual!=expected_crc) {
        snprintf(h,sizeof(h),"ERR tx_crc %08" PRIx32 "\n",actual);
        reply(h);return false;
    }
    return true;
}

/* layout 0 = Q[9:0],I[19:10]; layout 1 = I[9:0],Q[19:10]. */
static void tx_expand_iq8(unsigned n, unsigned layout) {
    uint8_t *raw=(uint8_t *)IQ_BUFFER;
    for(unsigned j=n;j>0;j--) {
        unsigned k=j-1;
        int16_t si=(int16_t)(int8_t)raw[2*k];
        int16_t sq=(int16_t)(int8_t)raw[2*k+1];
        uint32_t i10=(uint16_t)(si*4)&0x3ffu;
        uint32_t q10=(uint16_t)(sq*4)&0x3ffu;
        IQ_BUFFER[k]=layout ? (i10|(q10<<10)) : (q10|(i10<<10));
    }
}

/* ---- live TX register-experiment support (lab RE tool) ---- */
static unsigned s_loaded_words;

/* Transmit the currently loaded buffer continuously for hold_ms, re-triggering
 * each replay pass. Any SRAM-owner / format-register values POKEd beforehand
 * survive (this function does not touch them), so they can be swept live. */
static bool tx_hold(unsigned words, unsigned clock_arg, unsigned hold_ms) {
    if(!words || words>IQ_WORDS) return false;
    if(hold_ms>2000) hold_ms=2000;
    rx_ready=false;
    phy_stop_tx_tone(1);
    s3_tune(frequency_mhz);
    force_txon_mode(1,0,10);
    uint32_t ctrl=REG_READ(S3_TX_CTRL_REG)&0x7fffffffu;
    REG_WRITE(S3_TX_CTRL_REG,ctrl);
    ctrl=REG_READ(S3_TX_CTRL_REG);
    ctrl=(ctrl&0xffffc000u)|(words&0x3fffu);
    if(clock_arg==1)ctrl&=~BIT(15);else ctrl|=BIT(15);
    ctrl&=~BIT(19);
    ctrl=(ctrl&0xf00fffffu)|((words&0xffu)<<20);
    REG_WRITE(S3_TX_CTRL_REG,ctrl);
    int64_t start=esp_timer_get_time();
    REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
    while(esp_timer_get_time()-start < (int64_t)hold_ms*1000) {
        if(REG_READ(S3_TX_CTRL_REG)&BIT(18)) {   /* pass done -> retrigger */
            REG_WRITE(S3_TX_CTRL_REG,ctrl);
            REG_WRITE(S3_TX_CTRL_REG,ctrl|BIT(31));
        }
    }
    REG_CLR_BIT(S3_TX_CTRL_REG,BIT(31));
    REG_WRITE(S3_RX_CTRL_REG,0);
    force_txon_mode(0,0,10);
    rx_ready=false;
    prepare_rx();
    return true;
}

/* Whitelist: RF control block + SRAM owner reg only (peripheral regs, no CPU RAM). */
static bool reg_ok(unsigned a) {
    return (a>=0x60033c00u && a<=0x60033dffu) || a==SRAM_OWNER_REG;
}

/* Experimental TX using the REAL engine control register 0x60033d5c (the same
 * one capture() drives for RX) instead of the project's wrong 0x60033d64.
 * From the vendor RFTest disassembly the engine is shared: RX uses mode=0 with a
 * bit19 trigger pulse; a non-zero mode takes the other (DAC-out) branch. We
 * replicate capture()'s setup, force the PA on, load our I/Q from SRAM, and
 * sweep the mode field to find the DAC-out mode. Data must be preloaded (TXLOAD). */
/* Ported from vendor dactrig() setup (RFTest disasm ~0x40383754..0x403838a5),
 * excluding the SRAM vendor-ramp fill so host I/Q survives. Configures the
 * baseband->DAC datapath that our RX-only prepare_rx() never sets up. */
static void tx_setup_datapath(void) {
    REG_WRITE(0x600330d8u, REG_READ(0x600330d8u) & ~7u);
    REG_WRITE(0x600330dcu, REG_READ(0x600330dcu) & ~7u);
    REG_WRITE(0x600330e0u, REG_READ(0x600330e0u) & ~7u);
    REG_WRITE(0x600330e4u, REG_READ(0x600330e4u) & ~7u);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu) & ~2u);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu) & ~0x10u);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu) & 0x7fffffffu);
    REG_WRITE(0x60033080u, REG_READ(0x60033080u) | 0x10000000u);
    REG_WRITE(0x60033080u, (REG_READ(0x60033080u) & 0xffffff00u) | 12u);
    REG_WRITE(0x60033c34u, 31u);
    REG_WRITE(0x60033040u, 0x07060504u);
    REG_WRITE(0x60033044u, 0x00000908u);
    REG_WRITE(0x60033d04u, 0x04013000u);
    REG_WRITE(0x60033d90u, 0x000c2040u);
    REG_WRITE(0x60033c18u, REG_READ(0x60033c18u) & 0x7fffffffu);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu) | 0x800000u);
    REG_WRITE(0x600330d8u, REG_READ(0x600330d8u) | 32u);
    REG_WRITE(0x60033080u, REG_READ(0x60033080u) & 0xefffffffu);
    REG_WRITE(0x60033080u, REG_READ(0x60033080u) | 0x10000000u);
    REG_WRITE(0x60033084u, REG_READ(0x60033084u) | 0x80000000u);  /* helper 0x403835cc */
    REG_WRITE(0x60033080u, REG_READ(0x60033080u) & 0xefffffffu);
}

/* ===== Ported vendor dactrig: GDMA-fed DAC transmit =====
 * Verified live over JTAG against the RFTest firmware: the DAC replays a linked
 * ring of descriptors {dw0,buf,next} (dw0=0x80100100 => owner + 256-byte chunks)
 * whose ring address goes to 0x60033088 (masked to 20 bits). The datapath +
 * trigger register values all matched the running vendor firmware exactly. */
typedef struct { uint32_t dw0; uint32_t buf; uint32_t next; } tx_desc_t;
#define TX_DESC_BYTES 256u
#define TX_MAX_DESC   ((IQ_WORDS*4u + TX_DESC_BYTES - 1u)/TX_DESC_BYTES + 1u)
static tx_desc_t tx_desc[TX_MAX_DESC] __attribute__((aligned(16)));

/* TX analog of prepare_rx(): enable the TX mixer + DAC clock (the piece the
 * original TX path omitted — it only ever set RX mode). dac_rate is fuzzable. */
static bool s_rftest_inited=false;
static unsigned tx_chan(void){ return (frequency_mhz>=2412 && frequency_mhz<=2472)?(frequency_mhz-2412)/5+1:6; }
/* Put the PHY into the SAME TX state that wifiscwout uses (verified to radiate
 * ~49 dB). esp_phy_wifi_tx_tone(1,chan,0) keys the PA; the DAC replay then
 * feeds arbitrary I/Q on top of it. */
static void prepare_tx(unsigned dac_rate) {
    (void)dac_rate;
    rx_ready=false;
    if(!s_rftest_inited){ esp_phy_rftest_config(1); esp_phy_rftest_init(); s_rftest_inited=true; }
    esp_phy_tx_contin_en(true);
    esp_phy_wifi_tx_tone(1,tx_chan(),0);   /* key TX PA on this channel */
}
static void tx_teardown(void) {
    esp_phy_wifi_tx_tone(0,tx_chan(),0);   /* stop TX tone / unkey */
    esp_phy_tx_contin_en(false);
}

static bool tx_dma(unsigned words, unsigned hold_ms) {
    if(!words || words>IQ_WORDS) return false;
    if(hold_ms>500) hold_ms=500;
    unsigned nbytes=words*4u;
    unsigned ndesc=(nbytes+TX_DESC_BYTES-1u)/TX_DESC_BYTES;
    if(ndesc>TX_MAX_DESC) ndesc=TX_MAX_DESC;
    uint8_t *base=(uint8_t *)IQ_BUFFER;               /* our expanded 20-bit I/Q words */
    for(unsigned i=0;i<ndesc;i++) {
        unsigned len=TX_DESC_BYTES;
        if((i+1)*TX_DESC_BYTES>nbytes) len=nbytes-i*TX_DESC_BYTES;
        /* vendor descriptor style: owner|suc_eof, length=size=len (JTAG-verified
         * dw0=0xc0..0100). One-shot chain that terminates (next=0 on last). */
        tx_desc[i].dw0=0xc0000000u|((len&0xfffu)<<12)|(len&0xfffu);
        tx_desc[i].buf=(uint32_t)(base+i*TX_DESC_BYTES);
        tx_desc[i].next=(i+1u<ndesc)?(uint32_t)&tx_desc[i+1]:0u;
    }
    unsigned s_ndesc=ndesc;

    prepare_tx(s_dac_rate);          /* enable TX mixer + DAC clock */

    /* M start: gate 0x60033d14 bit1, wait bit0 */
    REG_WRITE(0x60033d14u, REG_READ(0x60033d14u)|2u);
    { int64_t t0=esp_timer_get_time();
      while(!(REG_READ(0x60033d14u)&1u) && esp_timer_get_time()-t0<2000){} }

    /* datapath config (matches vendor dactrig body, verified live) */
    REG_WRITE(0x600330d8u, REG_READ(0x600330d8u)&~7u);
    REG_WRITE(0x600330dcu, REG_READ(0x600330dcu)&~7u);
    REG_WRITE(0x600330e0u, REG_READ(0x600330e0u)&~7u);
    REG_WRITE(0x600330e4u, REG_READ(0x600330e4u)&~7u);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu)&~2u);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu)&~0x10u);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu)&0x7fffffffu);
    REG_WRITE(0x60033080u, REG_READ(0x60033080u)|0x10000000u);
    REG_WRITE(0x60033080u, (REG_READ(0x60033080u)&0xffffff00u)|12u);
    REG_WRITE(0x60033c34u, 31u);
    REG_WRITE(0x60033040u, 0x07060504u);
    REG_WRITE(0x60033044u, 0x00000908u);
    REG_WRITE(0x60033d04u, 0x04013000u);
    REG_WRITE(0x60033d90u, 0x000c2040u);
    REG_WRITE(0x60033c18u, REG_READ(0x60033c18u)&0x7fffffffu);
    REG_WRITE(0x60033c6cu, REG_READ(0x60033c6cu)|0x800000u);
    REG_WRITE(0x600330d8u, REG_READ(0x600330d8u)|32u);

    /* RF/baseband TX config copied from live vendor working state (JTAG diff).
     * Our RX-oriented prepare_rx()/force_txon left these at RX values. */
    REG_WRITE(0x6001c02cu, 0x374052edu);      /* TX gain */
    REG_WRITE(0x60033c60u, 0x00098000u);
    REG_WRITE(0x60033c6cu, 0x21c81d24u);
    REG_WRITE(0x60033c78u, 0x00000000u);

    /* bind DMA ring + trigger (bit28 pulse brackets enable).
     * Vendor idle state has 0x60033088 AND 0x60033094 = ring, 0x6003308c/90/98=0,
     * 0x6003309c=0x100. Ours had stale garbage in 8c/90/98 that misdirected the
     * DMA (current ptr ran outside our buffer) -> clear them to match. */
    uint32_t ring=((uint32_t)&tx_desc[0])&0xfffffu;
    /* Sustained: re-arm descriptors + re-trigger each pass for hold_ms. */
    int64_t start=esp_timer_get_time();
    do {
        for(unsigned i=0;i<s_ndesc;i++) tx_desc[i].dw0|=0x80000000u;  /* re-own */
        REG_WRITE(0x60033080u, REG_READ(0x60033080u)&~0x10000000u);
        REG_WRITE(0x6003308cu, 0);
        REG_WRITE(0x60033090u, 0);
        REG_WRITE(0x60033098u, 0);
        REG_WRITE(0x6003309cu, 0x00000100u);
        REG_WRITE(0x60033088u, ring);
        REG_WRITE(0x60033094u, ring);
        REG_WRITE(0x60033080u, REG_READ(0x60033080u)|0x10000000u);
        REG_WRITE(0x60033084u, REG_READ(0x60033084u)|0x80000000u);
        REG_WRITE(0x60033080u, REG_READ(0x60033080u)&~0x10000000u);
        for(volatile int w=0;w<2000;w++){}   /* let the pass run */
    } while(esp_timer_get_time()-start < (int64_t)hold_ms*1000);

    REG_WRITE(0x60033084u, REG_READ(0x60033084u)&~0x80000000u);
    force_txon_mode(0,0,10);
    rx_ready=false;
    prepare_rx();
    return true;
}

static void tx_engine(unsigned mode, unsigned source, unsigned pulse, unsigned hold_ms) {
    if(hold_ms>500) hold_ms=500;
    rx_ready=false;
    phy_stop_tx_tone(1);
    s3_tune(frequency_mhz);
    force_txon_mode(1,0,10);
    tx_setup_datapath();
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    REG_WRITE(S3_RX_CTRL_REG,0);
    REG_WRITE(0x60033d90u,rx_pack|((rx_pack+1)<<6)|((rx_pack+2)<<12)|((rx_pack+3)<<18)|(rx_agc<<24));
    REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    uint32_t ctrl=0x80000000u|(rx_wide<<17)|((source&0xffu)<<20)|((mode&0xfu)<<28)|(s_loaded_words&0x3fffu);
    int64_t start=esp_timer_get_time();
    do {
        REG_WRITE(S3_RX_CTRL_REG,ctrl);
        if(pulse){ REG_WRITE(S3_RX_CTRL_REG,ctrl|(1u<<19)); REG_WRITE(S3_RX_CTRL_REG,ctrl); }
        int64_t p0=esp_timer_get_time();
        while(!(REG_READ(S3_RX_CTRL_REG)&(1u<<18)) && esp_timer_get_time()-p0<2000){}
    } while(esp_timer_get_time()-start < (int64_t)hold_ms*1000);
    REG_WRITE(S3_RX_CTRL_REG,0);
    REG_WRITE(SRAM_OWNER_REG,owner);
    force_txon_mode(0,0,10);
    rx_ready=false;
    prepare_rx();
}

static void tx_fill_vendor_ramp(void) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<IQ_WORDS*4u;j++)p[j]=(uint8_t)((j>>3)+0x100u);
}

static void handle_command(char *line) {
    if(!strcmp(line,"TRANSPORT?")) {
        char answer[64];
        snprintf(answer,sizeof(answer),"TRANSPORT %s %u\n",
                 burst_serial_port()==BURST_SERIAL_UART?"UART":"USB",burst_serial_baud());
        reply(answer);return;
    }

    if(limits_command(line))return;
    if(gain_command(line))return;

    unsigned n,rate,crc,repeats;
    unsigned tx_clock,tx_flag,tx_layout;
    uint32_t tx_crc;
    char extra;
    uint64_t nonce;
    unsigned poke_addr,poke_val,hold_ms;
    bool iq8=false;

    if(!strncmp(line,"CAP16 ",6)){memcpy(line,"CAP20",5);iq8=true;}

    if(sscanf(line,"SYNC %" SCNu64 " %c",&nonce,&extra)==1) {
        char answer[48];snprintf(answer,sizeof(answer),"SYNC %" PRIu64 "\n",nonce);reply(answer);
    }
    else if(!strcmp(line,"TXLIMITS?")) {
        reply("TXLIMITS {\"words\":[1,16380],\"uploads\":[\"RAW32\",\"IQ8\"],"
              "\"clock_arg\":[0,1],\"flag_arg\":[0,1],\"iq_layout\":[0,1]}\n");
    }
    else if(!strcmp(line,"TXREGS?")) {
        char h[96];
        snprintf(h,sizeof(h),"TXREGS %08x %08x %08x\n",
            (unsigned)REG_READ(S3_TX_CTRL_REG),(unsigned)REG_READ(S3_RX_CTRL_REG),
            (unsigned)REG_READ(SRAM_OWNER_REG));
        reply(h);
    }
    else if(sscanf(line,"TXRAMP %u %u %u %c",&n,&tx_clock,&tx_flag,&extra)==3 &&
            n>=1 && n<=IQ_WORDS && tx_clock<=1 && tx_flag<=1) {
        tx_fill_vendor_ramp();
        tx_dac_replay(n,tx_clock,tx_flag);
    }
    else if(sscanf(line,"TXRAW %u %u %u %" SCNx32 " %c",
                   &n,&tx_clock,&tx_flag,&tx_crc,&extra)==4 &&
            n>=1 && n<=IQ_WORDS && tx_clock<=1 && tx_flag<=1) {
        if(tx_receive_payload((size_t)n*4u,tx_crc))
            tx_dac_replay(n,tx_clock,tx_flag);
    }
    else if(sscanf(line,"TXIQ8 %u %u %u %u %" SCNx32 " %c",
                   &n,&tx_clock,&tx_flag,&tx_layout,&tx_crc,&extra)==5 &&
            n>=1 && n<=IQ_WORDS && tx_clock<=1 && tx_flag<=1 && tx_layout<=1) {
        if(tx_receive_payload((size_t)n*2u,tx_crc)) {
            char lab[24];
            snprintf(lab,sizeof(lab),"%u MHz",frequency_mhz);
            display_waveform((const int8_t *)IQ_BUFFER,n,lab,true);
            tx_expand_iq8(n,tx_layout);
            tx_dac_replay(n,tx_clock,tx_flag);
        }
    }
    else if(sscanf(line,"TXLOAD %u %u %" SCNx32 " %c",&n,&tx_layout,&tx_crc,&extra)==3 &&
            n>=1 && n<=IQ_WORDS && tx_layout<=1) {
        if(tx_receive_payload((size_t)n*2u,tx_crc)) {
            tx_expand_iq8(n,tx_layout);
            s_loaded_words=n;
            char h[32];snprintf(h,sizeof(h),"LOADED %u\n",n);reply(h);
        }
    }
    else if(sscanf(line,"TXHOLD %u %u %c",&tx_clock,&hold_ms,&extra)==2 &&
            tx_clock<=1 && s_loaded_words) {
        tx_hold(s_loaded_words,tx_clock,hold_ms);
        char h[48];snprintf(h,sizeof(h),"TXHOLD %u %u\n",s_loaded_words,hold_ms);reply(h);
    }
    else if(sscanf(line,"TXTONE %u %c",&n,&extra)==1 && n<=1) {
        /* n=1 key a CW carrier on the current channel (esp_phy path, radiates),
         * n=0 stop it. Proven controllable transmitter. */
        if(n){ prepare_tx(0); reply("TXTONE on\n"); }
        else { tx_teardown(); rx_ready=false; prepare_rx(); reply("TXTONE off\n"); }
    }
    else if(sscanf(line,"TXVENDOR %u %u %u %u %c",&n,&tx_clock,&tx_flag,&repeats,&extra)==4 &&
            n>=1 && n<=IQ_WORDS) {
        /* Full TX bring-up, then call the REAL vendor dactrig, repeats times. */
        prepare_tx(s_dac_rate);
        uint32_t d64=0;
        for(unsigned r=0;r<repeats && r<2000;r++) dactrig(n,tx_clock,tx_flag,0);
        d64=REG_READ(S3_TX_CTRL_REG);
        rx_ready=false; prepare_rx();
        char h[64];snprintf(h,sizeof(h),"TXVENDOR %u done d64=%08x\n",n,(unsigned)d64);reply(h);
    }
    else if(sscanf(line,"TXLOADRAW %u %" SCNx32 " %c",&n,&tx_crc,&extra)==2 &&
            n>=1 && n<=IQ_WORDS) {
        /* upload n raw 32-bit words straight into IQ_BUFFER (no expansion),
         * so the host can experiment with the exact DAC sample format */
        if(tx_receive_payload((size_t)n*4u,tx_crc)) {
            s_loaded_words=n;
            char h[32];snprintf(h,sizeof(h),"LOADEDRAW %u\n",n);reply(h);
        }
    }
    else if(sscanf(line,"TXDMA %u %u %c",&hold_ms,&n,&extra)==2 && s_loaded_words) {
        s_dac_rate=n;               /* fuzzable DAC rate */
        tx_dma(s_loaded_words,hold_ms);
        char h[48];snprintf(h,sizeof(h),"TXDMA %u %u rate %u\n",s_loaded_words,hold_ms,n);reply(h);
    }
    else if(sscanf(line,"TXDMA %u %c",&hold_ms,&extra)==1 && s_loaded_words) {
        tx_dma(s_loaded_words,hold_ms);
        char h[48];snprintf(h,sizeof(h),"TXDMA %u %u\n",s_loaded_words,hold_ms);reply(h);
    }
    else if(sscanf(line,"TXENG %u %u %u %u %c",&n,&rate,&repeats,&hold_ms,&extra)==4 &&
            n<=15 && repeats<=1 && s_loaded_words) {
        /* TXENG <mode> <source> <pulse> <hold_ms> */
        tx_engine(n,rate,repeats,hold_ms);
        char h[48];snprintf(h,sizeof(h),"TXENG %u %u %u\n",n,rate,repeats);reply(h);
    }
    else if(sscanf(line,"PEEK %x %c",&poke_addr,&extra)==1 && reg_ok(poke_addr)) {
        char h[40];snprintf(h,sizeof(h),"PEEK %08x %08x\n",poke_addr,(unsigned)REG_READ(poke_addr));reply(h);
    }
    else if(sscanf(line,"POKE %x %x %c",&poke_addr,&poke_val,&extra)==2 && reg_ok(poke_addr)) {
        REG_WRITE(poke_addr,poke_val);reply("OK\n");
    }
    else if(sscanf(line,"RXRUN %u %u %u %u %c",&n,&rate,&repeats,&crc,&extra)==4 &&
            n>=256 && n<=IQ_WORDS && rate<=6 && repeats>0 && repeats<=1000 &&
            (crc==16 || crc==20)) {
        bool ok=true;
        for(unsigned j=0;j<repeats && ok;j++){ok=capture(n,rate,crc);vTaskDelay(1);}
        if(ok)reply("END\n");
    }
    else if(!strcmp(line,"CAPS")) {
        reply("CAPS RXLIMITS SERIALLEASE DUALSERIAL TXDAC TXRAW TXIQ8 "
              "TUNEEXT RX40 RX16 LPFANA GAIN HWAGC IQ8\n");
    }
    else if(sscanf(line,"BANDWIDTH %u %c",&n,&extra)==1 &&
            (!n || (n>=RX_BANDWIDTH_MIN && n<=RX_BANDWIDTH_MAX))) {
        rx_filter=rx_bandwidth_dcap(n);reply("OK\n");
    }
    else if(!strcmp(line,"LPF AUTO")){rx_filter=-1;reply("OK\n");}
    else if(sscanf(line,"LPF %u %c",&n,&extra)==1 && n<=63){rx_filter=n;reply("OK\n");}
    else if(!strcmp(line,"LPF?")){
        char answer[80];
        snprintf(answer,sizeof(answer),"LPF %d %u %u\n",rx_filter,
            rom_chip_i2c_readReg(0x67,0,4)&63,rom_chip_i2c_readReg(0x67,0,5)&63);
        reply(answer);
    }
    else if(!strcmp(line,"RANGE?")) {
        char answer[64];
        snprintf(answer,sizeof(answer),"RANGE %u %u 1\n",S3_FREQ_MIN,S3_FREQ_MAX);
        reply(answer);
    }
    else if(!strcmp(line,"INFO")) reply("S3SDR-TX 7 burst 16380\n");
    else if(sscanf(line,"FREQ %u %c",&n,&extra)==1 &&
            (n>=S3_FREQ_MIN && n<=S3_FREQ_MAX)) {
        frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
    }
    else if(((!strncmp(line,"CAP ",4) && sscanf(line,"CAP %u %u %c",&n,&rate,&extra)==2) ||
             (!strncmp(line,"CAP20 ",6) && sscanf(line,"CAP20 %u %u %c",&n,&rate,&extra)==2)) &&
            n>=256 && n<=IQ_WORDS && rate<=6) {
        capture(n,rate,!strncmp(line,"CAP20 ",6)?(iq8?16:20):0);
    }
    else reply("ERR command\n");
}

void app_main(void) {
    esp_log_level_set("*",ESP_LOG_NONE);
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=8192,.rx_buffer_size=8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE));
    prepare_rx();

    esp_log_level_set("*",ESP_LOG_NONE);
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
    burst_serial_init();
    display_init();
    display_status("ESP-SDR S3","idle");

    char line[128];
    int owner=-1;
    int64_t lease_deadline=0;
    for(;;) {
        if(esp_timer_get_time()>=lease_deadline)owner=-1;
        int status=burst_serial_poll_line(line,sizeof(line));
        if(!status){vTaskDelay(1);continue;}
        int port=burst_serial_port();
        if(owner>=0 && owner!=port){reply("ERR busy\n");continue;}
        if(status<0){reply("ERR command_length\n");continue;}
        owner=port;
        if(!strcmp(line,"RELEASE")) {
            reply("OK\n");owner=-1;
        } else {
            handle_command(line);
        }
        lease_deadline=esp_timer_get_time()+5000000;
    }
}
