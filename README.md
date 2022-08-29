# BHI260AP/BHI260AB/BHA260AB Sensor API, ported to Nordic nRF5 SDK (17.1.0)

(WORK IN PROGRESS DO NOT USE YET)

Orignal code by Bosch (https://github.com/BoschSensortec/BHY2-Sensor-API), ported to the Nordic nRF5 SDK version 17.1.0. Should work on any nRF5 SDK version, and has been tested on a custom nRF52840 board with the following connections

(work in progress)

BHY2* libraries are unchanged, the only ported portions are common.c/h and the euler.c example. Should be relatively easy to port other examples, with the exclusion of the CLI.

Please note that you will need to add the BHY2* files and common.c/h to a project, and add the necessary files to boot your board. If using one of Nordic's SDK boards, you could use the blinky example as starting point, and add the code in euler.c into main.c. Euler.c is written to never exit and use the NRF_LOG function to print euler angles once per second. The code is interrupt driven, and will need to have the Bosch INT pin properly connected to trigger the GPIOTE handler for parsing


# BHI260AP/BHI260AB/BHA260AB Sensor API

> This package contains BH260AP/BHI260AB/BHA260AB generically clustered as BHy2 sensor API

Product links
- [BHA260AB](https://www.bosch-sensortec.com/products/smart-sensors/bha260ab.html)
- [BHI260AB](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ab.html)
- [BHI260AP](https://www.bosch-sensortec.com/products/smart-sensors/bhi260ap/)

---
#### Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved
