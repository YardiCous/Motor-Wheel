#ifndef ROBOT_LOCAL_H
#define ROBOT_LOCAL_H
/* Copy to robot_local.h, edit measured values, then explicitly enable arming. */
#undef ROBOT_CALIBRATION_CONFIRMED
#define ROBOT_CALIBRATION_CONFIRMED 0
/* Example override (replace with your measured x4 quadrature count):
#undef ENCODER_COUNTS_PER_REV
#define ENCODER_COUNTS_PER_REV 960u
*/
#endif
