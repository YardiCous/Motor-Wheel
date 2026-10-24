/** @file selftest.h
 * @brief Motor-free on-target checks enabled by make TEST=1.
 */
#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdbool.h>

/** @brief Check format boundaries and safety latching before task startup. */
bool selftest_run(void);

#endif /* SELFTEST_H */

/*** End of file. ***/

