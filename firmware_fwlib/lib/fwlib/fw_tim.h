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

#ifndef ___FW_TIM_H___
#define ___FW_TIM_H___

#include "fw_conf.h"
#include "fw_types.h"
#include "fw_exti.h"

typedef enum
{
    TIM_TimerMode_16BitAuto         = 0x00,   // 16-bit auto-reload from [TH0,TL0](Timer0), [TH1,TL1](Timer1)
    TIM_TimerMode_16Bit             = 0x01,   // 16-bit no-auto-reload
    TIM_TimerMode_8BitAuto          = 0x02,   // 8-bit auto-reload from TH0(Timer0), TH1(Timer1)
    TIM_TimerMode_16BitAutoNoInt    = 0x03    // Uninterruptable 16-bit auto-reload, Timer0 only
} TIM_TimerMode_t;

int16_t TIM_Timer0n1_CalculateInitValue(uint16_t frequency, HAL_State_t freq1t, uint16_t limit);

/***************************** /
 * Timer 0
*/
#define TIM_Timer0_SetRunState(__STATE__)   SBIT_ASSIGN(TR0, __STATE__)
#define TIM_Timer0_SetGateState(__STATE__)  SFR_ASSIGN(TMOD, 3, __STATE__)
#define TIM_Timer0_SetFuncCounter           SFR_SET(TMOD, 2)
#define TIM_Timer0_SetFuncTimer             SFR_RESET(TMOD, 2)
// ON:FOSC, OFF:FOSC/12
#define TIM_Timer0_Set1TMode(__STATE__)     SFR_ASSIGN(AUXR, 7, __STATE__)
// Enable output on P3.5, when counter overflow, P3.5 switch voltage level
#define TIM_Timer0_SetOutput(__STATE__)     SFR_ASSIGN(INTCLKO, 0, __STATE__)
#define TIM_Timer0_SetMode(__TIM_TIMER_MODE__)  (TMOD = TMOD & ~(0x03 << 0) | ((__TIM_TIMER_MODE__) << 0))
#define TIM_Timer0_SetInitValue(__TH__, __TL__)  do{ TH0 = (__TH__); TL0 = (__TL__); }while(0)

void TIM_Timer0_Config(HAL_State_t freq1t, TIM_TimerMode_t mode, uint16_t frequency);


/***************************** /
 * Timer 1
*/
#define TIM_Timer1_SetRunState(__STATE__)   SBIT_ASSIGN(TR1, __STATE__)
#define TIM_Timer1_SetGateState(__STATE__)  SFR_ASSIGN(TMOD, 7, __STATE__)
#define TIM_Timer1_FuncCounter              SFR_SET(TMOD, 6)
#define TIM_Timer1_FuncTimer                SFR_RESET(TMOD, 6)
// ON:FOSC, OFF:FOSC/12
#define TIM_Timer1_Set1TMode(__STATE__)     SFR_ASSIGN(AUXR, 6, __STATE__)
// Enable output on P3.4, when counter overflow, P3.4 switch voltage level
#define TIM_Timer1_SetOutput(__STATE__)     SFR_ASSIGN(INTCLKO, 1, __STATE__)
#define TIM_Timer1_SetMode(__TIM_TIMER_MODE__)  (TMOD = TMOD & ~(0x03 << 4) | ((__TIM_TIMER_MODE__) << 4))
#define TIM_Timer1_SetInitValue(__TH__, __TL__)  do{ TH1 = (__TH__); TL1 = (__TL__); }while(0)

void TIM_Timer1_Config(HAL_State_t freq1t, TIM_TimerMode_t mode, uint16_t frequency);

#endif