#include <Arduino.h>

#ifndef STRUCTURE_H
#define STRUCTURE_H

struct imuData {
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    int16_t gyr_x, gyr_y, gyr_z;
    int16_t eul_heading, eul_roll, eul_pitch;
    int16_t qua_w, qua_x, qua_y, qua_z;
    int16_t lia_x, lia_y, lia_z;
    int16_t grv_x, grv_y, grv_z;
    int8_t  temp;
    uint8_t calib_stat;
  };

#endif 