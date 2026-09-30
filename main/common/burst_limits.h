#include "rx_bandwidth.h"
static bool limits_command(const char *line) {
    if(strcmp(line,"LIMITS?"))return false;
    char h[224];
    snprintf(h,sizeof(h),
        "LIMITS {\"gain\":[0,%u,1],\"bandwidth\":[13,69,1,0],"
        "\"rates\":[80000000,40000000,16000000],\"bits\":[8,10]}\n",
        gain_max());
    reply(h);return true;
}
