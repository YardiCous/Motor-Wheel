/** @file drv_lsm303.c
 * @brief LSM303DLHC / LSM303D detection, configuration and burst reads.
 *
 * Register values were checked against the ST datasheets:
 *   LSM303DLHC DocID018771 Rev 2 (lsm303dlhc.pdf in the course pack),
 *   LSM303D DocID023312 Rev 2 (DS_lsm303d.pdf; the text copy from ST was
 *   used because the course copy is a scan).
 *
 * Both chips: setting bit 7 of the register address makes a multi-byte
 * read auto-increment (DLHC section 5.1.1, LSM303D section 6.1.1).
 */
#include "drv_lsm303.h"

#include "../board_io.h"

#include <stddef.h>

/* LSM303DLHC: accelerometer at 0x19, magnetometer at 0x1E (Table 14). */
#define DRV_DLHC_ACCEL_ADDRESS 0x19u
#define DRV_DLHC_MAG_ADDRESS 0x1Eu
#define DRV_DLHC_IRA_REG_M 0x0Au /* reads 0x48 ('H') */
#define DRV_DLHC_IRB_REG_M 0x0Bu /* reads 0x34 ('4') */
#define DRV_DLHC_IRC_REG_M 0x0Cu /* reads 0x33 ('3') */
#define DRV_DLHC_IRA_VALUE 0x48u
#define DRV_DLHC_IRB_VALUE 0x34u
#define DRV_DLHC_IRC_VALUE 0x33u
#define DRV_DLHC_CTRL_REG1_A 0x20u
#define DRV_DLHC_CTRL_REG4_A 0x23u
#define DRV_DLHC_CRA_REG_M 0x00u
#define DRV_DLHC_CRB_REG_M 0x01u
#define DRV_DLHC_MR_REG_M 0x02u
#define DRV_DLHC_OUT_X_H_M 0x03u /* order X, Z, Y; high byte first */

/* LSM303D: one address for both sensors, 0x1D (SA0 high) or 0x1E (low). */
#define DRV_D_ADDRESS_SA0_HIGH 0x1Du
#define DRV_D_ADDRESS_SA0_LOW 0x1Eu
#define DRV_D_WHO_AM_I 0x0Fu
#define DRV_D_WHO_AM_I_VALUE 0x49u
#define DRV_D_CTRL1 0x20u
#define DRV_D_CTRL2 0x21u
#define DRV_D_CTRL5 0x24u
#define DRV_D_CTRL6 0x25u
#define DRV_D_CTRL7 0x26u
#define DRV_D_OUT_X_L_M 0x08u /* order X, Y, Z; low byte first */

/* Both chips: accelerometer output 0x28..0x2D, low byte first. */
#define DRV_OUT_X_L_A 0x28u
#define DRV_AUTO_INCREMENT 0x80u
#define DRV_SAMPLE_BYTES 6u
#define DRV_BYTE_BITS 8u
#define DRV_SIGN_BIT 32768
#define DRV_WORD_RANGE 65536

/* Scaling at +/-4 g. DLHC high-resolution output is 12 bits, left
 * justified, at 2 mg per count: raw / 16 * 2 = raw / 8 (exact, because the
 * low four bits are zero). LSM303D is 16 bits at 0.122 mg per count. */
#define DRV_DLHC_RAW_PER_MG 8
#define DRV_D_UMG_PER_COUNT 122
#define DRV_D_UMG_PER_MG 1000

/** @brief One register write in a configuration table. */
typedef struct
{
    uint8_t reg;
    uint8_t value;
} drv_setting_t;

/* DLHC accelerometer. CTRL_REG1_A = 0x77: ODR 400 Hz, normal power, X/Y/Z
 * on (Table 20). In high-resolution mode the internal filter bandwidth is
 * ODR/9 (Table 8): 400 Hz gives about 44 Hz, matching the LSM303D's 50 Hz
 * anti-alias filter below. The previous 100 Hz setting gave only 11 Hz,
 * which would smear short impact spikes. The task reads the newest sample
 * every 10 ms. CTRL_REG4_A = 0x98: BDU (no torn high/low bytes), +/-4 g so
 * a 2.5 g impact does not clip, HR (12-bit). */
static const drv_setting_t g_dlhc_accel[] = {
    {DRV_DLHC_CTRL_REG1_A, 0x77u},
    {DRV_DLHC_CTRL_REG4_A, 0x98u},
};

/* DLHC magnetometer: 15 Hz (Table 72), +/-1.3 gauss (Table 75),
 * continuous conversion (Table 78). Logged for comparison only. */
static const drv_setting_t g_dlhc_mag[] = {
    {DRV_DLHC_CRA_REG_M, 0x10u},
    {DRV_DLHC_CRB_REG_M, 0x20u},
    {DRV_DLHC_MR_REG_M, 0x00u},
};

/* LSM303D. CTRL1 = 0x6F: 100 Hz (Table 36), BDU, X/Y/Z on.
 * CTRL2 = 0xC8: anti-alias 50 Hz (Table 39), +/-4 g (Table 40).
 * CTRL5 = 0x70: magnetometer high resolution, 50 Hz (Table 47).
 * CTRL6 = 0x20: +/-4 gauss (Table 50). CTRL7 = 0x00: continuous
 * magnetometer, accelerometer filter bypassed (Tables 52 and 54). */
static const drv_setting_t g_d_settings[] = {
    {DRV_D_CTRL1, 0x6Fu}, {DRV_D_CTRL2, 0xC8u}, {DRV_D_CTRL5, 0x70u},
    {DRV_D_CTRL6, 0x20u}, {DRV_D_CTRL7, 0x00u},
};

static bool    detect(drv_lsm303_t * p_imu);
static bool    read_byte(uint8_t address, uint8_t reg, uint8_t * p_value);
static bool    apply(uint8_t address, const drv_setting_t * p_table,
                     uint32_t count);
static int16_t to_signed(uint8_t low, uint8_t high);

/** @brief Detection runs again on every call, so re-initialising after a
 * SENSOR_FAULT also copes with a module that was swapped or re-seated.
 */
bool drv_lsm303_init(drv_lsm303_t * p_imu)
{
    board_io_i2c_init();
    bool b_valid = detect(p_imu);

    if (b_valid && (DRV_LSM303_DLHC == p_imu->variant))
    {
        b_valid = apply(DRV_DLHC_ACCEL_ADDRESS, g_dlhc_accel,
                        sizeof(g_dlhc_accel) / sizeof(g_dlhc_accel[0]));
        b_valid =
            b_valid && (apply(DRV_DLHC_MAG_ADDRESS, g_dlhc_mag,
                              sizeof(g_dlhc_mag) / sizeof(g_dlhc_mag[0])));
    }
    else if (b_valid)
    {
        b_valid = apply((uint8_t)p_imu->accel_address, g_d_settings,
                        sizeof(g_d_settings) / sizeof(g_d_settings[0]));
    }
    else
    {
        /* Nothing answered with a known identity. */
    }

    return b_valid;
}

/** @brief One 6-byte burst; the high/low pairs are coherent because BDU is
 * set. Raw values stay int16_t: the chip's own format (Tribunal 1 exception).
 */
bool drv_lsm303_accel(const drv_lsm303_t * p_imu, int16_t * p_raw,
                      int16_t * p_mg)
{
    uint8_t bytes[DRV_SAMPLE_BYTES] = {0u};
    bool    b_valid                 = (DRV_LSM303_NONE != p_imu->variant) &&
                   (board_io_i2c_read((uint8_t)p_imu->accel_address,
                                      DRV_OUT_X_L_A | DRV_AUTO_INCREMENT,
                                      bytes, DRV_SAMPLE_BYTES));

    for (uint32_t axis = 0u; b_valid && (axis < DRV_LSM303_AXES); ++axis)
    {
        int16_t raw = to_signed(bytes[2u * axis], bytes[(2u * axis) + 1u]);
        int32_t mg  = (DRV_LSM303_DLHC == p_imu->variant)
                          ? (raw / DRV_DLHC_RAW_PER_MG)
                          : ((raw * DRV_D_UMG_PER_COUNT) / DRV_D_UMG_PER_MG);

        if (NULL != p_raw)
        {
            p_raw[axis] = raw;
        }

        if (NULL != p_mg)
        {
            /* |mg| is at most 4096 at +/-4 g. */
            p_mg[axis] = (int16_t)mg;
        }
    }

    return b_valid;
}

/** @brief DLHC registers are read one at a time: its magnetometer stores
 * X, Z, Y high-byte-first, and single reads avoid relying on its
 * auto-increment behaviour. Used only for logging, at 20 Hz.
 */
bool drv_lsm303_mag(const drv_lsm303_t * p_imu, int16_t * p_raw)
{
    uint8_t bytes[DRV_SAMPLE_BYTES] = {0u};
    bool    b_valid                 = (DRV_LSM303_NONE != p_imu->variant);

    if (DRV_LSM303_DLHC == p_imu->variant)
    {
        for (uint8_t index = 0u; b_valid && (index < DRV_SAMPLE_BYTES);
             ++index)
        {
            b_valid = read_byte(DRV_DLHC_MAG_ADDRESS,
                                (uint8_t)(DRV_DLHC_OUT_X_H_M + index),
                                &bytes[index]);
        }

        if (b_valid)
        {
            p_raw[0] = to_signed(bytes[1], bytes[0]); /* X */
            p_raw[1] = to_signed(bytes[5], bytes[4]); /* Y */
            p_raw[2] = to_signed(bytes[3], bytes[2]); /* Z */
        }
    }
    else if (b_valid)
    {
        b_valid = board_io_i2c_read((uint8_t)p_imu->mag_address,
                                    DRV_D_OUT_X_L_M | DRV_AUTO_INCREMENT,
                                    bytes, DRV_SAMPLE_BYTES);

        for (uint32_t axis = 0u; b_valid && (axis < DRV_LSM303_AXES); ++axis)
        {
            p_raw[axis] = to_signed(bytes[2u * axis], bytes[(2u * axis) + 1u]);
        }
    }
    else
    {
        /* No chip. */
    }

    return b_valid;
}

const char * drv_lsm303_name(const drv_lsm303_t * p_imu)
{
    const char * p_name = "none";

    if (DRV_LSM303_DLHC == p_imu->variant)
    {
        p_name = "LSM303DLHC";
    }
    else if (DRV_LSM303_D == p_imu->variant)
    {
        p_name = "LSM303D";
    }
    else
    {
        /* Not found. */
    }

    return p_name;
}

/** @brief DLHC first: three fixed ID bytes are a stronger signature than
 * one. Then LSM303D WHO_AM_I at 0x1D, then at 0x1E.
 */
static bool detect(drv_lsm303_t * p_imu)
{
    uint8_t ids[DRV_LSM303_ID_BYTES] = {0u};
    uint8_t ctrl                     = 0u;
    *p_imu                           = (drv_lsm303_t){0};
    bool b_dlhc =
        (read_byte(DRV_DLHC_MAG_ADDRESS, DRV_DLHC_IRA_REG_M, &ids[0])) &&
        (read_byte(DRV_DLHC_MAG_ADDRESS, DRV_DLHC_IRB_REG_M, &ids[1])) &&
        (read_byte(DRV_DLHC_MAG_ADDRESS, DRV_DLHC_IRC_REG_M, &ids[2])) &&
        (DRV_DLHC_IRA_VALUE == ids[0]) && (DRV_DLHC_IRB_VALUE == ids[1]) &&
        (DRV_DLHC_IRC_VALUE == ids[2]) &&
        (read_byte(DRV_DLHC_ACCEL_ADDRESS, DRV_DLHC_CTRL_REG1_A, &ctrl));

    if (b_dlhc)
    {
        p_imu->variant       = DRV_LSM303_DLHC;
        p_imu->accel_address = DRV_DLHC_ACCEL_ADDRESS;
        p_imu->mag_address   = DRV_DLHC_MAG_ADDRESS;
    }
    else
    {
        static const uint8_t g_d_addresses[] = {DRV_D_ADDRESS_SA0_HIGH,
                                                DRV_D_ADDRESS_SA0_LOW};
        uint32_t count = sizeof(g_d_addresses) / sizeof(g_d_addresses[0]);

        for (uint32_t index = 0u;
             (DRV_LSM303_NONE == p_imu->variant) && (index < count); ++index)
        {
            uint8_t who = 0u;

            if ((read_byte(g_d_addresses[index], DRV_D_WHO_AM_I, &who)) &&
                (DRV_D_WHO_AM_I_VALUE == who))
            {
                p_imu->variant       = DRV_LSM303_D;
                p_imu->accel_address = g_d_addresses[index];
                p_imu->mag_address   = g_d_addresses[index];
                ids[0]               = who;
                ids[1]               = g_d_addresses[index];
                ids[2]               = 0u;
            }
        }
    }

    for (uint32_t index = 0u; index < DRV_LSM303_ID_BYTES; ++index)
    {
        p_imu->id[index] = ids[index];
    }

    return (DRV_LSM303_NONE != p_imu->variant);
}

static bool read_byte(uint8_t address, uint8_t reg, uint8_t * p_value)
{
    return board_io_i2c_read(address, reg, p_value, 1u);
}

/** @brief Write each setting, then read it back to confirm. */
static bool apply(uint8_t address, const drv_setting_t * p_table,
                  uint32_t count)
{
    bool b_valid = true;

    for (uint32_t index = 0u; b_valid && (index < count); ++index)
    {
        uint8_t check = 0u;
        b_valid       = (board_io_i2c_write(address, p_table[index].reg,
                                            p_table[index].value)) &&
                  (read_byte(address, p_table[index].reg, &check)) &&
                  (p_table[index].value == check);
    }

    return b_valid;
}

/** @brief Two's complement bytes to int16_t without implementation-defined
 * conversion: values of 32768 and above are moved down by 65536.
 */
static int16_t to_signed(uint8_t low, uint8_t high)
{
    uint32_t word  = (uint32_t)low | ((uint32_t)high << DRV_BYTE_BITS);
    int32_t  value = (int32_t)word;
    value = (value >= DRV_SIGN_BIT) ? (value - DRV_WORD_RANGE) : value;
    return (int16_t)value;
}

/*** End of file. ***/

