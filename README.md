# e_oct_dice

A low-cost, open-source recreation of elemental octahedral dice in Genius Invokation TCG of Genshin Impact.

**This project is currently a Work-in-Progress (WIP). All materials in this repository is subject to change.**

> "Genshin Impact" is a trademark owned by COGNOSPHERE PTE. LTD. This project is a fanwork. Neither this project nor its authors have affiliations with COGNOSPHERE.

## Project structure

This project is divided into subdirectories listed below:

* [`firmware/`](firmware/): C firmware project for onboard MCU
* [`hardware/`](hardware/): Schematic and PCB designs
* [`physical/`](physical/): 3D solid shell designs
* [`assets/`](assets/): Other assets and non-code files

## Software requirements

* **Keil uVision 5** ([link](https://www.keil.com/product/)) to build firmware
* **STC-ISP 6** ([link](https://stcai.com/gjrj)) to program the MCU
* **EasyEDA** ([link](https://lceda.cn/)) to view, edit and prepare schematic and PCB designs
* **PTC Creo 5** ([link](https://www.ptc.com/en/products/creo)) to view, edit and prepare physical designs

## License

### Firmware code: GNU GPLv3 or later

```plain
Copyright (c) 2022-2023 Rong Bao (CSharperMantle) <baorong2005@126.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see https://www.gnu.org/licenses/.
```

You may also obtain a copy of this license at [LICENSE-GPL](LICENSE-GPL).

### Non-code files: CC BY-SA 4.0 International

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

You may also obtain a copy of the legal code of this license at [LICENSE-CC-BYSA-40](LICENSE-CC-BYSA-40).

## OSS used in this project

### [FwLib_STC8](https://github.com/IOsetting/FwLib_STC8)

**Author:** @IOsetting ([link](https://github.com/IOsetting))

**Description:** A lite firmware library for STC8G/STC8H series MCU

**License:** Apache 2.0 license

Adaptation and minor modifications have been made to this library to match project use cases. These modifications have been attributed in respective files. 

### [genshin-icon](https://github.com/cchampou/genshin-icon)

**Author:** @cchampou ([link](https://github.com/cchampou))

**Description:** A set of Genshin Impact icons in React components

**License:** MIT license

SVG files have been extracted from the React components with minor modification and patches.