/*
  Range   : Roll  : +/- 180 deg/sec
            Pitch : +/- 180 deg/sec
            Yaw   : +/- 180 deg/sec
  Scale   : Roll  : 1 = 1 deg/sec
            Pitch : 1 = 1 deg/sec
            Yaw   : 1 = 1 deg/sec
 */

#include <IMU.h>


cIMU    IMU;



uint8_t   err_code;
uint8_t   led_tog = 0;
uint8_t   led_pin = 13;
