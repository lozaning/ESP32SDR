#pragma once
#include <stdint.h>
#define RX_BANDWIDTH_MIN 13u
#define RX_BANDWIDTH_MAX 69u
static inline unsigned rx_bandwidth_phy_mode(unsigned mhz) {
    (void)mhz; return 0u;
}
typedef struct { uint8_t dcap,mhz; } rx_bandwidth_point_t;
static inline uint8_t rx_bandwidth_dcap(unsigned mhz) {
    static const rx_bandwidth_point_t cal[]={
        {0,69},{4,51},{8,45},{16,33},{24,25},{32,21},{48,16},{60,13}
    };
    unsigned count=sizeof(cal)/sizeof(cal[0]);
    if(!mhz)return 0;
    if(mhz>=cal[0].mhz)return cal[0].dcap;
    if(mhz<=cal[count-1].mhz)return cal[count-1].dcap;
    for(unsigned i=1;i<count;i++)if(mhz>=cal[i].mhz) {
        unsigned span=cal[i-1].mhz-cal[i].mhz;
        return cal[i-1].dcap+((cal[i].dcap-cal[i-1].dcap)*(cal[i-1].mhz-mhz)+span/2)/span;
    }
    return cal[count-1].dcap;
}
