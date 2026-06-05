#include <var.h>
#include <string.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

float calcAlphaEMA(float fn) {
    if (fn <= 0)
        return 1;
    // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
    const float c = cos(2 * float(M_PI) * fn);
    return c - 1 + sqrt(c * c - 4 * c + 3);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double handletime(float data) {
    if(data == 100) {
        return 10;
    } else if(data == 10) {
        return 100;
    } else {
        return 0;
    }
}


uint32_t getTeensySerial() {
	uint32_t num;
	num = HW_OCOTP_MAC0 & 0xFFFFFF;
	if (num < 10000000) {  
      num = num * 10;
      return num;
	}  else { 
      return num;
  }
}

//void(* resetFunc) (void) = 0;

void receiveEvent(int bytesReceived) {
    int _data = kire.read(); 
    Serial.print(_data);
    if(_data == 'a') {
        if (data != '3' || data != '4' || data != '0') {
            rpmAlter = !rpmAlter; 
      } } else if(_data == 'b') {
        if (data != '1' || data != '2' || data != '0') {
            rpmAlter_T = !rpmAlter_T; 
      } } else if(_data == 'R') {
        //resetFunc();
      } else if(_data != 10) {
        if(_data == data && elaspedTimeControlCounter < timeConstantControlCounter) {
          startTimeControlCounter = currentTimeControlCounter;
        } else {
          data = _data;
          startTimeControlCounter = currentTimeControlCounter;
        }
    }
}

