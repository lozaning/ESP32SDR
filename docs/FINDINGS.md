# ESP32-S3 DAC/capture engine — reverse-engineered register map
Source: EspRFTestTool_v5.2 / Bin/RF_TEST_BIN/ESP32-S3_RFTest_V113_86797624_20251111.bin
Disassembled with xtensa-esp-elf-objdump; IRAM segment @ 0x40378000. Trigger
routine spans ~0x403836c0..0x40383b74 (see vendor_trig_function.asm).

## Registers actually used by the vendor engine
- 0x60033d5c  CONTROL   (bit31=enable, bit19=trigger pulse, bit18=done, bit16/15=rate, bit17=wide, bits20-27=source, bits28-31=mode, low14=sample count)
- 0x60033d60  STATUS/COUNT (low14 = completed sample count, bit14 flag) — used to verify completion
- 0x600c101c  SRAM owner (low 4 bits; set to 4 during operation, cleared to release)
- 0x60033d90  format/pack  (vendor writes 0xc2040)
- 0x60033d04  writes 0x4013000
- 0x60033040/0x60033044  write 0x07060504 / 0x00000908
- 0x60033c18  read-modify-write (mask in low bits)
- 0x60033c34  writes 31
- 0x60033080  set bit 0x10000000
- 0x600330d8/dc/e0/e4  clear low 3 bits (&~7)
- 0x60033c6c  clear bits (&~3, &~0x11)
- 0x6001c02c  burst gain (already known)

## Why the project's TX (tx_dac_replay) never worked
1. It writes control register **0x60033d64**, which the vendor firmware NEVER touches.
   The real control register is **0x60033d5c** (same one capture() already uses).
   => every TX trigger write landed on an unused/wrong register.
2. It performs almost none of the ~two dozen setup writes above (SRAM/DMA
   addressing, format, clocks). The buffer routing to the DAC is never configured.
3. SRAM owner should be 4 (capture value), not 3.

## What still needs pinning down for a working TX
- The exact mode/source bit values that select DAC-out (TX) vs ADC-in (RX) in
  the 0x60033d5c control word (bits 20-31), and whether TX uses the bit19 pulse.
- Fastest way to get ground truth: flash this vendor RF-test firmware, start a
  DAC transmission via its CLI (e.g. wifiscwout / esp_tx), and read the live
  0x60033xxx register values over the S3 built-in JTAG (openocd) — then replicate.

## Live JTAG capture of vendor working TX state (2026-09-30)
Registers (vendor RFTest running, TX configured) — replicated in our tx_dma():
- 0x60033088 = descriptor ring addr & 0xfffff   (also mirrored at 0x60033094)
- 0x60033080 = 0x0000000c ; 0x60033084 = 0x80000000 ; 0x60033d14 = 1 (gate)
- 0x60033040=0x07060504 ; 0x60033044=0x00000908 ; 0x60033d90=0x000c2040
- 0x60033c60=0x00098000 ; 0x60033c6c=0x21c81d24 ; 0x60033c78=0 ; 0x6001c02c(gain)=0x374052ed
- 0x60033d08 = 0x000ba220 (a 2nd ring @0x3fcba220, 1024-byte descriptors) — NOT yet replicated
- Descriptor format: {dw0,buf,next}, dw0=0x80100100 (owner=1,len=256,size=256), contiguous 256B buffers.
- FE DMA current-pointer regs seen live: 0x6003308c/90/94/98/9c (e.g. 0x60033094=ring). Semantics not fully decoded.

STATUS: our tx_dma() now makes ALL known datapath+RF registers match the vendor's working state (JTAG-verified), descriptor ring built over our 0x3fcd0000 I/Q buffer, DMA appears active, and TX keys (RX sees ~+6dB DC rise). BUT the commanded I/Q tone does not cleanly radiate yet. Remaining suspects: (1) exact DAC sample FORMAT (our tx_expand_iq8 20-bit packing may be wrong; 0x60033d90=0xc2040 defines packing), (2) the 2nd ring 0x60033d08, (3) FE-DMA data-path regs 0x60033090-9c, (4) freq-specific RF cal in 0x6001c02c. Method that works: JTAG-diff our live state vs vendor and copy differences.
