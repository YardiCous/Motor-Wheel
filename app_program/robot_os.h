/** @file robot_os.h
 * @brief Micro T-Kernel objects, core affinity, and application startup.
 */
#ifndef ROBOT_OS_H
#define ROBOT_OS_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Create all objects before starting any of the seven tasks. */
void robot_os_run(void);
/** @brief Nonblocking binary command bridge, called by the network owner. */
bool robot_os_submit(uint8_t payload);
/** @brief Retrieve the latest bounded 28-byte telemetry message. */
bool robot_os_telemetry(uint8_t * p_packet);
/** @brief Report connection bits without inhibiting autonomous control. */
void robot_os_network(bool b_wifi, bool b_mqtt);
/** @brief Consume one binary acknowledgement for robot/ack. */
bool robot_os_ack(uint8_t * p_ack);

#endif /* ROBOT_OS_H */

/*** End of file. ***/

