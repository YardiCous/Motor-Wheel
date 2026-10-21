#include "hardware/i2c.h"
#include "robot.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static struct {
    unsigned model : 2;
    unsigned address : 7;
    unsigned failures : 2;
    unsigned calibration_count : 7;
    signed calibration_sum : 19; /* 100 * +/-1800 tenths of a degree */
    signed pitch_zero : 12;
    signed pitch : 12;
    signed height_um : 24;
    unsigned peak_mm : 10;
    unsigned distance : 24;
    unsigned state : 3;
    unsigned generation : 8;
    uint16_t state_at, retry_at;
} terrain;
static bool read_register(uint8_t address, uint8_t reg, uint8_t *data, size_t size) {
    return i2c_write_timeout_us(i2c1, address, &reg, 1, true, IMU_I2C_TIMEOUT_US) == 1 &&
           i2c_read_timeout_us(i2c1, address, data, size, false, IMU_I2C_TIMEOUT_US) == (int)size;
}
static bool write_register(uint8_t address, uint8_t reg, uint8_t data) {
    uint8_t bytes[] = {reg, data};
    return i2c_write_timeout_us(i2c1, address, bytes, 2, false, IMU_I2C_TIMEOUT_US) == 2;
}
static bool detect(void) {
    uint8_t id;
    if (IMU_MODEL == 0 || IMU_MODEL == 1) {
        for (unsigned address = 0x68; address <= 0x69; ++address) {
            if (read_register(address, 0x75, &id, 1) && id == 0x68 &&
                write_register(address, 0x6b, 0x01) && write_register(address, 0x1a, 0x03) &&
                write_register(address, 0x1c, 0x08) && write_register(address, 0x19, 9)) {
                terrain.model = 1;
                terrain.address = address;
                return true;
            }
        }
    }
    if (IMU_MODEL == 0 || IMU_MODEL == 2) {
        /* GY-511: LSM303DLHC accelerometer, 100 Hz, +/-4g, high resolution. */
        if (read_register(0x19, 0x0f, &id, 1) && id == 0x33 && write_register(0x19, 0x20, 0x57) &&
            write_register(0x19, 0x23, 0x98)) {
            terrain.model = 2;
            terrain.address = 0x19;
            return true;
        }
    }
    return false;
}
static bool acceleration(int *x, int *y, int *z) {
    uint8_t data[6];
    if (!read_register(terrain.address, terrain.model == 1 ? 0x3b : 0xa8, data, 6))
        return false;
    int values[3];
    for (unsigned i = 0; i < 3; ++i) {
        int16_t raw = terrain.model == 1 ? (int16_t)((data[2 * i] << 8) | data[2 * i + 1])
                                         : (int16_t)((data[2 * i + 1] << 8) | data[2 * i]);
        values[i] = terrain.model == 1 ? (int)raw * 1000 / 8192 : (raw / 16) * 2;
    }
    *x = values[0];
    *y = values[1];
    *z = values[2];
    return true;
}
void imu_reset(void) {
    terrain.height_um = 0;
    terrain.peak_mm = 0;
    terrain.state = IMU_LEVEL;
    terrain.generation = robot_reset_generation();
    robot_state_t state;
    robot_snapshot(&state);
    uint32_t distance = 0;
    bits_get(state.motion.bytes, 8, 18, 24, &distance);
    terrain.distance = distance;
}
void imu_init(void) {
    memset(&terrain, 0, sizeof(terrain));
    terrain.state = IMU_INIT;
    terrain.retry_at = (uint16_t)(robot_now() - 1000u);
}
void imu_step(void) {
    uint16_t now = robot_now();
    if (terrain.state == IMU_INIT || terrain.state == IMU_FAULT) {
        if (elapsed_ms(now, terrain.retry_at) < 1000)
            return;
        terrain.retry_at = now;
        if (!detect()) {
            robot_imu_ready(false);
            robot_raise_fault(F_IMU);
            return;
        }
        terrain.state = IMU_CALIBRATING;
        terrain.calibration_count = 0;
        terrain.calibration_sum = 0;
        return;
    }
    int x, y, z;
    if (!acceleration(&x, &y, &z)) {
        if (terrain.failures < 3)
            terrain.failures++;
        if (terrain.failures == 3) {
            terrain.state = IMU_FAULT;
            robot_imu_ready(false);
            robot_raise_fault(F_IMU);
        }
        return;
    }
    terrain.failures = 0;
    int pitch = (int)lroundf(
        atan2f((float)(-x * IMU_PITCH_SIGN), sqrtf((float)y * y + (float)z * z)) * 572.9578f);
    robot_state_t state;
    robot_snapshot(&state);
    uint32_t vl, vr, distance, stamp;
    bits_get(state.motion.bytes, 8, 0, 9, &vl);
    bits_get(state.motion.bytes, 8, 9, 9, &vr);
    bits_get(state.motion.bytes, 8, 18, 24, &distance);
    bits_get(state.motion.bytes, 8, 45, 16, &stamp);
    if (!state.motion_valid || elapsed_ms(now, (uint16_t)stamp) > MOTION_MAX_AGE_MS)
        return;
    if (terrain.state == IMU_CALIBRATING) {
        if (abs((int)vl - 250) > 10 || abs((int)vr - 250) > 10 || abs(z) < 700 || abs(z) > 1300) {
            terrain.calibration_count = 0;
            terrain.calibration_sum = 0;
            return;
        }
        terrain.calibration_sum += pitch;
        terrain.calibration_count++;
        if (terrain.calibration_count == IMU_CALIBRATION_SAMPLES) {
            terrain.pitch_zero = terrain.calibration_sum / IMU_CALIBRATION_SAMPLES;
            imu_reset();
            robot_imu_ready(true);
            robot_clear_cause(F_IMU);
        }
        return;
    }
    if (terrain.generation != robot_reset_generation())
        imu_reset();
    int64_t magnitude = (int64_t)x * x + (int64_t)y * y + (int64_t)z * z;
    if (magnitude > (int64_t)IMU_IMPACT_MG * IMU_IMPACT_MG) {
        terrain.state = IMU_IMPACT;
        robot_raise_fault(F_IMPACT);
        return;
    }
    if (terrain.state == IMU_IMPACT)
        return; /* reset by acknowledged lifecycle generation */
    int filtered = (terrain.pitch * 7 + pitch - terrain.pitch_zero) / 8;
    terrain.pitch = filtered;
    uint32_t raw_delta = (distance - terrain.distance) & 0xffffffu;
    int delta = raw_delta < 0x800000u ? (int)raw_delta : (int)raw_delta - 0x1000000;
    terrain.distance = distance;
    if (abs(delta) > 30) {
        robot_raise_fault(F_ENCODER);
        return;
    }
    if (terrain.state == IMU_LEVEL && filtered > 50 && controller_may_drive(&state.control)) {
        terrain.state = IMU_CLIMBING;
        terrain.state_at = now;
        terrain.height_um = 0;
        terrain.peak_mm = 0;
        (void)robot_send_event(EV_HUMP_START, 0);
    }
    if (terrain.state == IMU_CLIMBING || terrain.state == IMU_CRESTING ||
        terrain.state == IMU_DESCENDING) {
        int height = terrain.height_um + (int)lroundf(delta * 1000.0f * sinf(filtered / 572.9578f));
        if (height < -8388608 || height > 1023000) {
            robot_raise_fault(F_IMU);
            return;
        }
        terrain.height_um = height;
        if (height > (int)terrain.peak_mm * 1000)
            terrain.peak_mm = height / 1000;
        if (terrain.state == IMU_CLIMBING && elapsed_ms(now, terrain.state_at) > 3000) {
            terrain.state = IMU_LEVEL;
            (void)robot_send_event(EV_HUMP_END, 0);
            return;
        }
        if (terrain.state == IMU_CLIMBING && filtered < 20)
            terrain.state = IMU_CRESTING;
        if (filtered < -50)
            terrain.state = IMU_DESCENDING;
        if (terrain.state == IMU_DESCENDING && abs(filtered) < 20) {
            robot_set_peak(terrain.peak_mm);
            (void)robot_send_event(EV_HUMP_END, terrain.peak_mm);
            terrain.state = IMU_LEVEL;
        }
        if (elapsed_ms(now, terrain.state_at) > 10000)
            robot_raise_fault(F_IMU);
    }
}
