
/*
 *  ======== ccfg.c ========
 *  Customer Configuration for CC26xx and CC13xx devices.  This file is used to
 *  configure Boot ROM, start-up code, and SW radio behaviour.
 *
 *  By default, driverlib startup_files/ccfg.c settings are used.  However, if
 *  changes are required there are two means to do so:
 *
 *    1.  Remove this file and copy driverlib's startup_files/ccfg.c file in
 *        its place.  Make all changes to the file.  Changes made are local to
 *        the project and will not affect other projects.
 *
 *    2.  Perform changes to driverlib startup_files/ccfg.c file.  Changes
 *        made to this file will be applied to all projects.  This file must
 *        remain unmodified.
 */
// Эта хрень не работает, оставил, чтобы долбаный RTOS знал что DCDC отключен
#define SET_CCFG_MODE_CONF_DCDC_RECHARGE 0x1 // Do not use the DC/DC during recharge in powerdown
#define SET_CCFG_MODE_CONF_DCDC_ACTIVE 0x1   // Do not use the DC/DC during active mode

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(startup_files/ccfg.c)
