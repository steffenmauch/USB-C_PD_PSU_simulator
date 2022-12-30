# USB-C_PD_PSU_simulator

USB-C PD power supply simulator based on STUSB4710 with CP2112 USB/I2C interface chip

### Introduction

- USB-C power delivery powers supply simulator based on STUSB4710
- can be used to simulated non conformant power supplies
- e.g. output different voltage than the one being negotiated
- manipulate offered contracts to simulate specific power supplies
- layout and schematic has been realized with KiCad 6
- correspondig software is written with python 3
- communication with STUSB4710 is realized with SiLabs 2112 USB to I2C/SMBUS interface chip

### Usage


### Background information for STUSB4710
- 'dm00664189-the-stusb4500-software-programing-guide-stmicroelectronics.pdf'
- 'STUSB47xx_register_map_public.pdf'
- 'STSW-STUSB004 - Quick Start - v1.0.pdf'
- 'TNxxx - STUSB4500 NVM Description_V1.1.pdf' and 'TNxxx - STUSB4700 NVM Description.pdf' which is part of 'STSW-STUSB003' and STSW-STUSB004'


### Background information for CP2112
- API information can be found in 'AN496.pdf'


### License
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)