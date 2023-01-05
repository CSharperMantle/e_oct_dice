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

#ifndef ___FW_UART_H___
#define ___FW_UART_H___

#include "fw_conf.h"
#include "fw_types.h"
#include "fw_exti.h"

int16_t UART_Timer_InitValueCalculate(uint32_t sysclk, HAL_State_t _1TMode, uint32_t baudrate);


/**************************************************************************** /
 * UART1
*/

typedef enum
{
    UART1_BaudSource_Timer1 = 0x00,
    UART1_BaudSource_Timer2 = 0x01,
} UART1_BaudSource_t;

/**
 * Alternative ports
 * 
 * The ports for STC8G1K08-8Pin and STC8G1K08A are different:
 * 00 - P3.0 P3.1, 01 - P3.2 P3.3, 10 - P5.4 P5.5, 11 - n/a
*/
typedef enum
{
    UART1_AlterPort_P30_P31      = 0x00,
    UART1_AlterPort_P36_P37      = 0x01,
    UART1_AlterPort_P16_P17      = 0x10,
    UART1_AlterPort_P43_P44      = 0x11,
} UART1_AlterPort_t;

#define UART1_SetRxState(__STATE__)         SBIT_ASSIGN(REN, __STATE__)
#define UART1_ClearTxInterrupt()            SBIT_RESET(TI)
#define UART1_ClearRxInterrupt()            SBIT_RESET(RI)
#define UART1_WriteBuffer(__DATA__)         (SBUF = (__DATA__))
#define UART1_SetFrameErrDetect(__STATE__)  SFR_ASSIGN(PCON, 6, __STATE__)
#define UART1_SetBaudSource(__BAUD_SRC__)   SFR_ASSIGN(AUXR, 0, __BAUD_SRC__)
/**
 * Mode0: Synchronous shift serial mode, baudrate is fixed, provided by SYSCLK
 * Baud = (UART_M0x6 = 0)? (SYSCLK/12) : (SYSCLK/2)
*/
#define UART1_ConfigMode0FixedSyncSerial(__STATE__)        do{ SM0=0; SM1=0; SFR_ASSIGN(AUXR, 5, __STATE__);}while(0)
#define UART1_SetMode0Baudx6(__STATE__)     SFR_ASSIGN(AUXR, 5, __STATE__)
/**
 * Mode2: 9-bit UART mode, baudrate is fixed, provided by SYSCLK
 * Baud = (SMOD = 0)? (SYSCLK/64) : (SYSCLK/32)
*/
#define UART1_ConfigMode2Fixed9bitUart(__STATE__)            do{ SM0=1; SM1=0; SFR_ASSIGN(PCON, 7, __STATE__);}while(0)
#define UART1_SetTimer1Mode2Baudx2(__STATE__)   SFR_ASSIGN(PCON, 7, __STATE__)
/**
 * Alternative port selection: P30/P31, P36/P37, P16/P17, P43/P44
*/
#define UART1_SwitchPort(__ALTER_PORT__)    (P_SW1 = P_SW1 & ~(0x03 << 6) | ((__ALTER_PORT__) << 6))
/**
 * Mode1:
 *   8-bit UART; 
 *   dynamic baud-rate; 
 *   10-bit frame: 1 start, 8 data(lsb), 1 stop; 
 *   clocked by Timer1 or Timer2
 * 
*/
void UART1_Config8bitUart(UART1_BaudSource_t baudSource, HAL_State_t _1TMode, uint32_t baudrate);


#endif