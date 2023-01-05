// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fw_sys.h"

/**
 * An approximate estimate of instruction cycles in one second, may vary in
 * different compilers even differnt versions, adjust it if you find the
 * delay too slow or fast.
*/
#if defined (__SDCC_SYNTAX_FIX)
    #define __CLK_REF 10000
#elif defined (SDCC) || defined (__SDCC)
    #define __CLK_REF 9000
#elif defined __CX51__
    #define __CLK_REF 10000
#endif

__CODE uint8_t  clkdiv      = ((__CONF_CLKDIV == 0)? 1 : __CONF_CLKDIV);
__CODE uint16_t ticks_ms    = (__CONF_FOSC / ((__CONF_CLKDIV == 0)? 1 : __CONF_CLKDIV) / __CLK_REF);
__CODE uint8_t  ticks_us    = (__CONF_FOSC / ((__CONF_CLKDIV == 0)? 1 : __CONF_CLKDIV) / __CLK_REF / 1000);


void SYS_Delay(uint16_t t)
{
    uint16_t i;
    do
    {
        i = ticks_ms;
        while (--i);
    } while (--t);
}

void SYS_DelayUs(uint16_t t)
{
    uint8_t i;
    do
    {
        i = ticks_us;
        while (--i);
    } while (--t);
}

uint32_t SYS_GetSysClock(void)
{
    return ((uint32_t)__CONF_FOSC) / clkdiv;
}
