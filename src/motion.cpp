#include <var.h>
#include <Arduino.h>

void motion(char _data) {
  if(_data == '0') { //TODO Implemtation
    rpmAlter = false;
    rpmAlter_T = false;
    //Serial5.write(0);
    //Serial5.write(128);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
  }  else if (_data == '1') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? 61: 63);    //Value RIGHT: 0(Stop) - 63(MAX) CW
    //Serial5.write(rpmAlter == 0 ? 191 : 191); //Value LEFT: 128(Stop) - 191(MAX) CW
    //-------------------------
    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215: 250);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 205: 240);
  } else if(_data == '2') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? BRW : BRD); //Value RIGHT: 64(Stop) - 127(MAX) CCW
    //Serial5.write(rpmAlter == 0 ? BLW: BLD);  //Value LEFT: 192(Stop) - 255(MAX) CCW
    
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 : 240);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 211 : 249); 
  } else if (_data == '3') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TRR:FRD);
    //Serial5.write(rpmAlter_T == 0 ?TRL:BLW);

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 82 : 148);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80: 146);
  } else if (_data == '4') {
    rpmAlter = false;
    //Serial5.write(rpmAlter_T == 0 ?TLR:BRW);
    //Serial5.write(rpmAlter_T == 0 ?TLL:FLD);
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter_T == 0 ? 83 :150);
    analogWrite(pwmPin_R, rpmAlter_T == 0 ? 80 :147);
  }  else if(_data == '5') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? FRW: FRD); //TODO: TO BE changed 
    //Serial5.write(TLL);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 204:245);
    analogWrite(pwmPin_R, 150);  
  } else if(_data == '6') {
    rpmAlter_T = false;
    //Serial5.write(TRR);
    //Serial5.write(rpmAlter == 0 ? FLW : FLD);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 215 :250);
  } else if(_data == '7') {
    rpmAlter_T = false;
    //Serial5.write(TLR);
    //Serial5.write(rpmAlter == 0 ? BLW: BLD);
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, rpmAlter == 0 ? 209 : 251);
    analogWrite(pwmPin_R, 150); 
  } else if(_data == '8') {
    rpmAlter_T = false;
    //Serial5.write(rpmAlter == 0 ? BRW : BRD);
    //Serial5.write(TRL);
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);
    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, rpmAlter == 0 ? 202 :245);
  } else {} 
}

