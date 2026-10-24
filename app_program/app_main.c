/** @file app_main.c
 * @brief Preserve the pre-existing kernel ABI; implementation is in robot_os.
 */
#include "app_main.h"

#include "robot_os.h"

/** @brief The legacy entry symbol must match the kernel's usermain hook. */
int usermain (void)
{
    robot_os_run();
    return 0;
}

/*** End of file. ***/

