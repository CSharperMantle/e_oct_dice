// Copyright 2021 IOsetting <iosetting(at)outlook.com>
// Copyright 2022-2023 Rong Bao (CSharperMantle) <baorong2005@126.com>
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

int putchar(int dat) {
    UART1_WriteBuffer(dat);
    while(!TI);
    UART1_ClearTxInterrupt();
    return dat;
}
