/** @file config_device.h
 * @brief Device allocation for the CSC20 application.
 * Copyright (C) 2020-2021 by Ken Sakamura.
 * This software is distributed under the T-License 2.2.
 */
#ifndef CONFIG_DEVICE_H
#define CONFIG_DEVICE_H

#define DEVCNF_USE_SER 1

/* The line task owns ADC directly through board_io. */
#define DEVCNF_USE_ADC 0

/* The terrain task owns I2C1 directly through board_io. */
#define DEVCNF_USE_IIC 0

#endif /* CONFIG_DEVICE_H */

/*** End of file. ***/

