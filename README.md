# e_oct_dice

This project is a fanwork, presenting a low-cost, open-source recreation of elemental octahedral dice in Genius Invokation TCG of Genshin Impact.

## Project structure

This project is divided into subdirectories listed below:

* `firmware_fwlib`: Keil C firmware project for onboard MCU
* `hardware`: Schematic and PCB designs
* `physical`: Outer solid shell designs

## Software requirements

* **Keil uVision v5** to build firmware
* **STC-ISP v6** to program the MCU
* **LCEDA (嘉立创EDA)** to view, edit and prepare schematic and PCB designs
* **PTC Creo 5** to view, edit and prepare physical designs

## License

### Project code: BSD 3-Clause

```plain
Copyright (c) 2022-2023 Rong Bao (CSharperMantle) <baorong2005@126.com>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

You may also obtain a copy of this license at [LICENSE-BSD](LICENSE-BSD).

### PCB, schematic, physical and anything other than code: CC BY-SA 4.0 International

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

You may also obtain a copy of the legal code of this license at [LICENSE-CC-BYSA-40](LICENSE-CC-BYSA-40).

## OSS used in this project

### [FwLib_STC8](https://github.com/IOsetting/FwLib_STC8)

**Author:** IOsetting
**Description:** A lite firmware library for STC8G/STC8H series MCU
**License:** Apache 2.0 license
