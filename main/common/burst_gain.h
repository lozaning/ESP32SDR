extern void force_rx_gain(unsigned,unsigned,unsigned);
#define BURST_GAIN_REG 0x6001c02cu
typedef enum { GAIN_MANUAL, GAIN_HARDWARE } burst_gain_mode_t;
static burst_gain_mode_t gain_mode=GAIN_HARDWARE;
static unsigned gain_code=40;
static unsigned gain_max(void) {
    unsigned maximum=(REG_READ(BURST_GAIN_REG)>>8)&127u;
    return maximum<=82u ? maximum : 0u;
}
static void gain_apply(void) {
    bool manual=gain_mode==GAIN_MANUAL;
    if(gain_code>gain_max())gain_code=gain_max();
    force_rx_gain(manual,gain_code,0);
}
static bool gain_command(const char *line) {
    unsigned code;char extra;
    if(!strcmp(line,"GAIN?")) {
        char h[64];snprintf(h,sizeof(h),"GAIN %s %d 0 %u %u\n",
            gain_mode==GAIN_HARDWARE?"HARDWARE":"MANUAL",
            gain_mode==GAIN_HARDWARE?-1:(int)gain_code,gain_max(),
            (unsigned)((REG_READ(BURST_GAIN_REG)>>23)&1));
        reply(h);return true;
    }
    if(!strcmp(line,"GAIN HARDWARE")) {gain_mode=GAIN_HARDWARE;}
    else if(sscanf(line,"GAIN MANUAL %u %c",&code,&extra)==1 && code<=gain_max()) {
        gain_mode=GAIN_MANUAL;gain_code=code;
    } else return false;
    gain_apply();reply("OK\n");return true;
}
