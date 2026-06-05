#include <var.h>
#include <maf.h>
#include <string.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <structure.h>
#include <i2c_driver.h>
#include <TeensyThreads.h>
#include <i2c_driver_wire.h>

int SLAVE_ADDRESS = 0x72;

uint8_t BNO_ADDR =       0x28;  // I2C address of BNO
uint8_t ACC_DATA_X_LSB = 0x08;  // BNO register Acceleration Data X LSB
uint8_t CALIB_STAT =     0x35;  // BNO register SYS Calib Status <7:6>, GYR Calib Status <5:4>, ACC Calib Status <3:2>, MAG Calib Status <1:0>
uint8_t OPR_MODE   =     0x3D; // BNO register Operation Mode <3:0>
uint8_t NDOF       =     0x0C;

imuData imu;

int FLW = 0;
int FRW = 0;
int BLW = 0;
int BRW = 0;

int FLD = 0;
int FRD = 0;
int BLD = 0;
int BRD = 0;

int TRR = 0;
int TRL = 0;
int TLR = 0;
int TLL = 0;

int pwmPin_R = 14; //22
int dirPin_R = 20; //22
int pwmPin_L = 23; //11
int dirPin_L = 22; //11


unsigned int addressFLW =  0;
unsigned int addressFRW =  2;
unsigned int addressBLW =  4;
unsigned int addressBRW =  6;

unsigned int addressFLD =  8;
unsigned int addressFRD = 10;
unsigned int addressBLD = 12;
unsigned int addressBRD = 14;

unsigned int addressTRR = 16;
unsigned int addressTRL = 18;
unsigned int addressTLR = 20;
unsigned int addressTLL = 22;

int encoderPin_1_L = 7;
int encoderPin_2_L = 8;

int encoderPin_1_R = 5;
int encoderPin_2_R = 4;

volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
long lastencoderValue_L = 0;
int lastMSB_L = 0;
int lastLSB_L = 0;

volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;
long lastencoderValue_R = 0;
int lastMSB_R = 0;
int lastLSB_R = 0;

//DIR and PWM Data
int    delayBrake = 50;
int pin_Emergency = 15;

//Handle Print Vsriable
bool     printAlter = false;
bool emergencyAlter = true;
bool  systemCounter = false;

//Control Variable
char  data = '0';
int rpmAlter_T = 0;
int rpmAlter = 0;
int _dirData = 0;

//Time Variable
float timeConstant = 100; //MilliSecond 100 | 10 ONLY -> Function-> handletime
float startTime, elaspedTime = 0, currentTime;

//Time Setup Control Counter
float timeConstantControlCounter = 1000; //Chnage This As per Trail 
float startTimeControlCounter, elaspedTimeControlCounter = 0, currentTimeControlCounter;

void setup() {
  //Serial Begin
  Serial.begin(115200);
  //Serial5.begin(115200);//TODO: For Simplified Serial
  kire.begin(SLAVE_ADDRESS); 
  //initWire();
  
   //Motor Pin Setup
   pinMode(dirPin_L, OUTPUT);
   pinMode(pwmPin_L, OUTPUT);
   
   pinMode(dirPin_R, OUTPUT);
   pinMode(pwmPin_R, OUTPUT);

  //initializeBNO055();
  pinMode(pin_Emergency, INPUT_PULLDOWN);

  //Time Setup
  startTime = millis();

  //Time Setup Control Counter
  startTimeControlCounter = millis();


  digitalWrite(dirPin_L, LOW);
  digitalWrite(dirPin_R, LOW);
  digitalWrite(pwmPin_L, LOW);
  digitalWrite(pwmPin_R, LOW);

  //Encoder Setup
  pinMode(encoderPin_1_L, INPUT); 
  pinMode(encoderPin_2_L, INPUT);

  digitalWrite(encoderPin_1_L, HIGH); 
  digitalWrite(encoderPin_2_L, HIGH); 
  attachInterrupt(encoderPin_1_L, updateEncoder_L, CHANGE); 
  attachInterrupt(encoderPin_2_L, updateEncoder_L, CHANGE);

  pinMode(encoderPin_1_R, INPUT); 
  pinMode(encoderPin_2_R, INPUT);

  digitalWrite(encoderPin_1_R, HIGH); 
  digitalWrite(encoderPin_2_R, HIGH); 
  attachInterrupt(encoderPin_1_R, updateEncoder_R, CHANGE); 
  attachInterrupt(encoderPin_2_R, updateEncoder_R, CHANGE);

  //Read Permanent Data 
  readEEPROM();

  Serial5.write(0);
  Serial5.write(192);
  /*
  kire.setClock(400 * 1000);
  kire.begin();                // initialize I2C
  bno_write(BNO_ADDR, OPR_MODE, NDOF);
  delay(1000);

  threads.addThread(thread_func);
  */
}

void loop() { 

  kire.onReceive(receiveEvent);

  //Handle Fliter
  static SMA<20> filter_L;
  double rpm_L = 0;
  uint16_t avgRPM_L = 0;

  static SMA<20> filter_R;
  double rpm_R = 0;
  uint16_t avgRPM_R = 0;

  int emergency = analogRead(pin_Emergency);//TODO: Comment if emergency is removed 
  
  //Handle  Time
  currentTime = millis(); 
  elaspedTime = currentTime - startTime;

  //Time Setup Control Counter
  currentTimeControlCounter = millis(); 
  elaspedTimeControlCounter = currentTimeControlCounter - startTimeControlCounter;

  
  //Handle Serial Commands
  if(Serial.available()) {
    if(systemCounter == false) {
      int _data = Serial.read();
      if(_data == char('m')) {
        Serial.print(getTeensySerial());
         Serial.print(" | ");
         Serial.println("Motion Module");
         data = char('0');
      } else if(_data == char('s')) {
        systemCounter = true;
        printAlter = false; //TODO: Can be Removed for fast testing
        data = '0';//TODO: Can be Removed for fast testing
        printSetting();
      } else if(_data == char('p')) {
        printAlter =  !printAlter;
      } else if(_data == char('a')) {
        if (data != '3' || data != '4' || data != '0') {
            rpmAlter = !rpmAlter; 
      } } else if(_data == char('b')) {
        if (data != '1' || data != '2' || data != '0') {
            rpmAlter_T = !rpmAlter_T; 
      } } else if(_data != 10) {
        data = _data;
        if(_data == data && elaspedTimeControlCounter < timeConstantControlCounter) {
          startTimeControlCounter = currentTimeControlCounter;
        } else {
          data = _data;
          startTimeControlCounter = currentTimeControlCounter;
        }
      }  
    } else {
      String _data = Serial.readString();
      _data = _data.remove(_data.length()-1, 1);
      if(_data.length() == 1 && _data == "s") {
        Serial.println("Chaging to Control Mode");
        systemCounter = false;
      } else if(_data.length() >= 1) {
        updatedEEPROM(_data);
      }
    } 
  }

  if(elaspedTimeControlCounter > timeConstantControlCounter) {
     data = '0'; //Commenting for Testing //TODO:
    startTimeControlCounter = currentTimeControlCounter;
  }
  
  if(emergency > 900) { 
    rpmAlter = false;
    //Serial5.write(0);
    //Serial5.write(128);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
    data = '0';
  } else if (emergency < 900) {
    motion(data);
  } 
  
  if (elaspedTime > timeConstant) {
    startTime = currentTime;

    //RPM Calculation
    rpm_L = ((abs(encoderValue_L)* 60 * handletime(timeConstant)) / 4000.00); // Change According to the time constant.
    rpm_R = ((abs(encoderValue_R)* 60 * handletime(timeConstant)) / 4000.00);
    //rpm = (No of Pluses/Total Pules) * 1sec 
    //Total Pulse = Pulse * 4 where is changes in both the phases
    //SEC -> MilliSec 60*100 -> TimeConstant will be 100
    //SEC -> MilliSec 60*1000 -> TimeConstant will be 10
    
    avgRPM_L = filter_L(rpm_L);
    avgRPM_R = filter_R(rpm_R);


    if(printAlter == true) {  
      Serial.print(data);
      Serial.print(" | ");
      //Serial.print(rpmAlter);
      //Serial.print(" | ");
      //Serial.print(rpmAlter_T);
      //Serial.print(" | ");
      Serial.print(emergency);
      Serial.print(" | ");
      Serial.print(avgRPM_L);
      Serial.print(" | ");
      Serial.print(avgRPM_R);
      Serial.print(" | ");
      Serial.print(1.0/16*imu.eul_heading);
      Serial.print(" | ");
      Serial.print(1.0/16*imu.eul_roll);
      Serial.print(" | ");
      Serial.print(1.0/16*imu.eul_pitch);
      Serial.print(" | ");
      Serial.println(imu.temp);
      /*Serial.print(" | ");
      Serial.print((s.calib_stat >> 6) & 3);
      Serial.print(" | ");
      Serial.print((s.calib_stat >> 4) & 3);
      Serial.print(" | ");
      Serial.print((s.calib_stat >> 2) & 3);
      Serial.print(" | ");
      Serial.println((s.calib_stat >> 0) & 3); */
   }

   encoderValue_L = encoderValue_R = 0;

  }

}

