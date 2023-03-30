/*
 * Copyright (c) 2022-2023 Rong Bao (CSharperMantle) <baorong2005@126.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see https://www.gnu.org/licenses/.
 */

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
