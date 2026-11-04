/** @file drv_lsm303.h
 * @brief GY-511 driver: LSM303DLHC or LSM303D, detected at run time.
 *
 * The lecturer warned that some GY-511 boards carry an LSM303D instead of
 * the LSM303DLHC. They differ in I2C addresses, registers and scaling, and
 * 0x1E is used by both (DLHC magnetometer, or LSM303D with SA0 low), so
 * the chip is identified by its ID registers, never by address alone.
 * All bus access goes through board_io (I2C1, GP2 SDA / GP3 SCL).
 */
#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdbool.h>
#include <stdint.h>

#define DRV_LSM303_NONE 0u
#define DRV_LSM303_DLHC 1u
#define DRV_LSM303_D 2u

#define DRV_LSM303_AXES 3u
#define DRV_LSM303_ID_BYTES 3u

/** @brief Which chip was found, and where. */
typedef struct
{
    uint32_t variant : 2;       /* DRV_LSM303_NONE, DLHC or D */
    uint32_t accel_address : 7; /* 7-bit I2C address */
    uint32_t mag_address : 7;
    /* Identity registers as read, for the boot print. Raw chip bytes. */
    uint8_t id[DRV_LSM303_ID_BYTES];
} drv_lsm303_t;

/** @brief Reset I2C1, identify the chip and write its configuration.
 * Each configuration register is read back to prove the write landed.
 * @return false if no supported chip answers or a write did not stick.
 */
bool drv_lsm303_init(drv_lsm303_t * p_imu);
/** @brief Read one acceleration sample: raw chip counts and milli-g.
 * Both arrays hold chip X, Y, Z. Either pointer may be NULL.
 */
bool drv_lsm303_accel(const drv_lsm303_t * p_imu, int16_t * p_raw,
                      int16_t * p_mg);
/** @brief Read raw magnetometer counts in chip X, Y, Z order. */
bool drv_lsm303_mag(const drv_lsm303_t * p_imu, int16_t * p_raw);
/** @brief Printable chip name for the boot banner. */
const char * drv_lsm303_name(const drv_lsm303_t * p_imu);

#endif /* DRV_LSM303_H */

/*** End of file. ***/

