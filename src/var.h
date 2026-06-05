#include <string.h>
#include <Arduino.h>
#include <structure.h>

#ifndef VAR_H
#define VAR_H

extern imuData imu;
extern uint8_t BNO_ADDR;  // I2C address of BNO
extern uint8_t ACC_DATA_X_LSB;  // BNO register Acceleration Data X LSB
extern uint8_t CALIB_STAT;  // BNO register SYS Calib Status <7:6>, GYR Calib Status <5:4>, ACC Calib Status <3:2>, MAG Calib Status <1:0>
extern uint8_t OPR_MODE; // BNO register Operation Mode <3:0>
extern uint8_t NDOF;

extern char data;

//---------------------
extern int pwmPin_L;
extern int dirPin_L;

//Control Pin R
extern int pwmPin_R;
extern int dirPin_R;


//Encoder Pin L
extern int encoderPin_1_L;
extern int encoderPin_2_L;
//extern int encoderPin3;

//Encoder Variable L
extern volatile int lastEncoded_L;
extern volatile long encoderValue_L;
extern long lastencoderValue_L;
extern int lastMSB_L;
extern int lastLSB_L;
 
//Encoder Pin R
extern int encoderPin_1_R;
extern int encoderPin_2_R;
//extern int encoderPin3;

//Encoder Variable R
extern volatile int lastEncoded_R;
extern volatile long encoderValue_R;
extern long lastencoderValue_R;
extern int lastMSB_R;
extern int lastLSB_R;

extern float startTimeControlCounter; 
extern float elaspedTimeControlCounter;
extern float currentTimeControlCounter;
extern float timeConstantControlCounter;

//Pid Variable
extern int rpmAlter; 
extern int rpmAlter_T; 
//Speed -> EEPROM Address
extern int FLW;
extern int FRW;
extern int BLW;
extern int BRW;

extern int FLD;
extern int FRD;
extern int BLD;
extern int BRD;

extern int TRR;
extern int TRL;
extern int TLR;
extern int TLL;

//Speed Variable
extern unsigned int addressFLW;
extern unsigned int addressFRW;
extern unsigned int addressBLW;
extern unsigned int addressBRW;

extern unsigned int addressFLD;
extern unsigned int addressFRD;
extern unsigned int addressBLD;
extern unsigned int addressBRD;

extern unsigned int addressTRR;
extern unsigned int addressTRL;
extern unsigned int addressTLR;
extern unsigned int addressTLL;

//float PID(float input);
void readEEPROM();
void printSetting();
void updatedEEPROM(String _data);
int handleEEPROMwrite(String _data, unsigned int address);

void updateEncoder_L();
void updateEncoder_R();

void motion(char _data);

uint32_t getTeensySerial();
float calcAlphaEMA(float fn);
double handletime(float data);
//void updatedEEPROM(String _data);
void receiveEvent(int bytesReceived);
long map(long x, long in_min, long in_max, long out_min, long out_max);

void thread_func();
void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data);
void bno_read_multiple(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t length);

#endif