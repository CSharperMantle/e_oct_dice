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

#include "fw_uart.h"
#include "fw_tim.h"
#include "fw_sys.h"


int16_t UART_Timer_InitValueCalculate(uint32_t sysclk, HAL_State_t _1TMode, uint32_t baudrate)
{
    uint32_t value;
    value = sysclk / (4 * baudrate);
    if (!_1TMode)
        value = value / 12;
    if (value > 0xFFFF)
        return 0;
    else
        return 0xFFFF - value + 1;
}

/**************************************************************************** /
 * UART1
*/

void _UART1_ConfigDynUart(UART1_BaudSource_t baudSource, HAL_State_t _1TMode, int16_t init)
{
    UART1_SetBaudSource(baudSource);
    // Timer1 configuration. Mode0 only, mode2 is covered by mode0 so it is unnecessary.
    if (baudSource == UART1_BaudSource_Timer1)
    {
        TIM_Timer1_Set1TMode(_1TMode);
        TIM_Timer1_SetMode(TIM_TimerMode_16BitAuto);
        TIM_Timer1_SetInitValue(init >> 8, init & 0xFF);
        TIM_Timer1_SetRunState(HAL_State_ON);
    }
    // Timer2 configuration
    else
    {
        // Timer2: 1T mode and initial value. prescaler is ignored, no interrupt.
        TIM_Timer2_Set1TMode(_1TMode);
        TIM_Timer2_SetInitValue(init >> 8, init & 0xFF);
        TIM_Timer2_SetRunState(HAL_State_ON);
    }
}
void UART1_Config8bitUart(UART1_BaudSource_t baudSource, HAL_State_t _1TMode, uint32_t baudrate)
{
    uint16_t init;
    uint32_t sysclk;
    SM0=0; SM1=1;
    sysclk = SYS_GetSysClock();
    init = UART_Timer_InitValueCalculate(sysclk, _1TMode, baudrate);
    _UART1_ConfigDynUart(baudSource, _1TMode, init);
}

int putchar(int dat) {
    UART1_WriteBuffer(dat);
    while(!TI);
    UART1_ClearTxInterrupt();
    return dat;
}
