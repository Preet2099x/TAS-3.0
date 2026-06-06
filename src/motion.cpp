#include <var.h>
#include <Arduino.h>

static unsigned long rampStartTime = 0;
static bool rampActive = false;
static int lastCommand = 0;
static int rampCommand = 0;


int applyStartRamp(int targetPWM, int currentCommand)
{
    if(rampActive && currentCommand == rampCommand)
    {
        unsigned long elapsed = millis() - rampStartTime;

        if(elapsed >= 1000)
        {
            rampActive = false;
            return targetPWM;
        }

        return 50 + ((targetPWM - 50) * elapsed) / 1000;
    }

    if(lastCommand == 0 &&
       (currentCommand == 1 || currentCommand == 2))
    {
        rampStartTime = millis();
        rampCommand = currentCommand;
        rampActive = true;

        return 50;
    }

    return targetPWM;
}

void motion(int _data) {
  if(_data == 0) 
  { 
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);

    rampActive = false;
    rampCommand = 0;
  }  
  else if (_data == 1) {

    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);

    int pwm = applyStartRamp(230, _data);

    analogWrite(pwmPin_R, pwm);
    analogWrite(pwmPin_L, pwm);

  } 
  else if(_data == 2) 
  {
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);

    int pwm = applyStartRamp(230, _data);

    analogWrite(pwmPin_R, pwm);
    analogWrite(pwmPin_L, pwm);
  } 
  else if (_data >= 11 && _data <= 20) 
  {
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);

    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, 150);

  } 
  else if (_data >= 21 && _data <= 30) 
  {
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);

    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, 150);
  }  
  else if(_data >= 111 && _data <= 120) 
  {
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    analogWrite(pwmPin_L, 200);
    analogWrite(pwmPin_R, 150);  

  } 
  else if(_data >= 121 && _data <= 130) 
  {
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, 200);
  } 
  else if(_data >= 211 && _data <= 220) 
  {
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    analogWrite(pwmPin_L, 200);
    analogWrite(pwmPin_R, 150); 
  } 
  else if(_data >= 221 && _data <= 230) 
  {
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    analogWrite(pwmPin_L, 150);
    analogWrite(pwmPin_R, 200);
  } 
  lastCommand = _data;
}

