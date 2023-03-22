#include "pd_sleep.h"
#include "fwlib/fw_hal.h"

void sleep_10ms_pd(unsigned char count) small {
    do {
        WKTCL = 0x18;
        WKTCH = 0x80;
        RCC_SetPowerDownMode(HAL_State_ON);
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        WKTCH = 0x00;
    } while (--count);
}

void sleep_1ms_pd(unsigned char count) small {
    do {
        WKTCL = 0x01;
        WKTCH = 0x80;
        RCC_SetPowerDownMode(HAL_State_ON);
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        WKTCH = 0x00;
    } while (--count);
}
