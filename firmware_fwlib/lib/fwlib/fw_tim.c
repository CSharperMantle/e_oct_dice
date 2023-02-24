// Copyright 2021 IOsetting <iosetting(at)outlook.com
// Copyright 2022-2023 Rong Bao (CSharperMantle) <baorong2005@126.com>>
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

#include "fw_tim.h"
#include "fw_sys.h"

#if 0 /* DISABLED FOR UNUSED FUNCTIONS */

/**
 * Calculate the initial value of Timer0 & Timer1 counter
 * - If the frequency is too high, it will return the value of `limit`, so the timer
 *   will run in the highest frequency
 * - If the frequency is too low, it will return 0, so the timer will run in the 
 *   lowest possible frequency
*/
int16_t TIM_Timer0n1_CalculateInitValue(uint16_t frequency, HAL_State_t freq1t, uint16_t limit)
{
    uint32_t value = SYS_GetSysClock();
    if (!freq1t)
        value = value / 12;
    value = value / frequency;
    if (value > limit)
        return 0;
    else
        return limit - value;
}

void TIM_Timer0_Config(HAL_State_t freq1t, TIM_TimerMode_t mode, uint16_t frequency)
{
    uint16_t init;
    TIM_Timer0_Set1TMode(freq1t);
    TIM_Timer0_SetMode(mode);
    if (mode == TIM_TimerMode_8BitAuto)
    {
        init = TIM_Timer0n1_CalculateInitValue(frequency, freq1t, 0xFF);
        TIM_Timer0_SetInitValue(init & 0xFF, init & 0xFF);
    }
    else
    {
        init = TIM_Timer0n1_CalculateInitValue(frequency, freq1t, 0xFFFF);
        TIM_Timer0_SetInitValue(init >> 8, init & 0xFF);
    }
}

void TIM_Timer1_Config(HAL_State_t freq1t, TIM_TimerMode_t mode, uint16_t frequency)
{
    uint16_t init;
    TIM_Timer1_Set1TMode(freq1t);
    TIM_Timer1_SetMode(mode);
    if (mode == TIM_TimerMode_8BitAuto)
    {
        init = TIM_Timer0n1_CalculateInitValue(frequency, freq1t, 0xFF);
        TIM_Timer1_SetInitValue(init & 0xFF, init & 0xFF);
    }
    else
    {
        init = TIM_Timer0n1_CalculateInitValue(frequency, freq1t, 0xFF);
        TIM_Timer1_SetInitValue(init >> 8, init & 0xFF);
    }
}

#endif /* 0 */
