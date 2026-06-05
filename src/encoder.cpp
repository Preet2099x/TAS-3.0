#include <var.h>
#include <Arduino.h>


void updateEncoder_L() {
  int MSB = digitalRead(encoderPin_1_L); //MSB = most significant bit
  int LSB = digitalRead(encoderPin_2_L); //LSB = least significant bit

  int encoded = (MSB << 1) | lastLSB_L; //converting the 2 pin value to single number
  int sum  = (lastEncoded_L << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_L ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_L --;

  lastEncoded_L = lastEncoded_L; //store this value for next time
}

void updateEncoder_R() {
  int MSB = digitalRead(encoderPin_1_R); //MSB = most significant bit
  int LSB = digitalRead(encoderPin_2_R); //LSB = least significant bit

  int encoded = (MSB << 1) | lastLSB_R; //converting the 2 pin value to single number
  int sum  = (lastEncoded_R << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_R ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_R --;

  lastEncoded_R = lastEncoded_R; //store this value for next time
}