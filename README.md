# BHI260AP/BHI260AB/BHA260AB Sensor API, ported to Nordic nRF5 SDK (17.1.0)

Orignal code by Bosch (https://github.com/BoschSensortec/BHY2-Sensor-API), ported to the Nordic nRF5 SDK version 17.1.0. Should work on any nRF5 SDK version, and has been tested on a custom nRF52840 board, but should work on any nRF52 device

# BHI260 connections

BHI260AB connected to BMM150 via I2C, as the Shuttle board

Need the following #define with the nRF52 GPIO pin, then connect it to the following BHI260 pins

| #define       | BHI260 pin   |
| ------------- | ------------ |
| BSP_MEMS_CS   | HCSB pin 34  |
| BSP_SPI_CLK   | HSCX pin 33  |
| BSP_SPI_MISO  | HSDO pin 32  |
| BSP_SPI_MOSI  | HSDX pin 11  |
| BSP_MEMS_INT  | HIRQ pin 10  |
| BSP_MEMS_nRESET | HSDX pin 9   |
     
# Ported code

BHY2* libraries are unchanged, the only ported portions are common.c/h and the euler.c example. Should be relatively easy to port other examples, with the exclusion of the CLI.

No other example has been ported

# Euler.c example

Please note that you will need to add the BHY2* files and common.c/h to a project, and add the necessary files to boot your board. If using one of Nordic's SDK boards, you could use the blinky example as starting point, and add the code in euler.c into main.c. Euler.c is written to never exit and use the NRF_LOG function to print euler angles once per second. The code is interrupt driven, and will need to have the Bosch INT pin properly connected to trigger the GPIOTE handler for parsing


# Embedded system optimization 

I left the Bosch libraries unchanged on purpose, to ensure future compatibility. The current code is inefficient for an embedded device with a fixed SPI connection, and still uses the interface abstraction from Bosch. For performance and memory reasons, should be rewritten eliminating the bhy2.c/h layer and optimizing the bhy_hif.c/h layer to hardcode the SPI interface

#  (original Bosch readme) BHI260AP/BHI260AB/BHA260AB Sensor API

> This package contains BH260AP/BHI260AB/BHA260AB generically clustered as BHy2 sensor API

Product links
- [BHA260AB](https://www.bosch-sensortec.com/products/smart-sensors/bha260ab.html)
- [BHI260AB](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ab.html)
- [BHI260AP](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ap/)

---
#### Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved
