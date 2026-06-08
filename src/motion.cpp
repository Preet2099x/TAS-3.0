#include <var.h>
#include <Arduino.h>

static unsigned long rampStartTime = 0;
static bool rampActive = false;
static int lastCommand = 0;
static int rampCommand = 0;

static int currentPwmL = 0;
static int currentPwmR = 0;

static unsigned long stopRampStartTime = 0;
static bool stopRampActive = false;

static int stopStartPwmL = 0;
static int stopStartPwmR = 0;

static bool headingHoldActive = false;
static unsigned long headingCaptureStart = 0;

static float targetHeading = 0;

static float headingIntegral = 0;
static float previousHeadingError = 0;
static unsigned long previousPidTime = 0;

const float Kp = 0.5f;
const float Ki = 0.0f;
const float Kd = 0.01f;


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

float normalizeHeadingError(float error)
{
    while(error > 180.0f) error -= 360.0f;
    while(error < -180.0f) error += 360.0f;

    return error;
}



void motion(int _data) {
if(_data == 0)
{
  headingHoldActive = false;
  headingCaptureStart = 0;
  headingIntegral = 0;
  previousPidTime = 0;

    if(!stopRampActive)
    {
        stopRampStartTime = millis();

        stopStartPwmL = currentPwmL;
        stopStartPwmR = currentPwmR;

        stopRampActive = true;
    }

    unsigned long elapsed = millis() - stopRampStartTime;

    if(elapsed >= 20) //20ms Ramp Down
    {
        currentPwmL = 0;
        currentPwmR = 0;

        analogWrite(pwmPin_L, 0);
        analogWrite(pwmPin_R, 0);

        digitalWrite(dirPin_L, LOW);
        digitalWrite(dirPin_R, LOW);

        stopRampActive = false;
        rampActive = false;
        rampCommand = 0;

        lastCommand = 0;
        return;
    }

    float progress = elapsed / 20.0f; //20ms Ramp Down
    float scale = 1.0f - (progress * progress);

    currentPwmL = stopStartPwmL * scale;
    currentPwmR = stopStartPwmR * scale;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);

    lastCommand = 0;
    return;
}
  stopRampActive = false;

  if (_data == 1)
  {

    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);

    int pwm = applyStartRamp(230, _data);

    if(!headingHoldActive)
    {
      if(headingCaptureStart == 0)
        headingCaptureStart = millis();

      if(millis() - headingCaptureStart >= 200)
      {
        targetHeading = currentHeading;
        headingHoldActive = true;
        headingIntegral = 0;
        previousHeadingError = 0;
        previousPidTime = millis();
      }
    }

    if(headingHoldActive)
    {
      float error = normalizeHeadingError(targetHeading - currentHeading);

      unsigned long now = millis();

      float dt = (now - previousPidTime) / 1000.0f;

      if(dt > 0.001f)
      {
        float derivative = (error - previousHeadingError) / dt;

        float correction = Kp * error + Kd * derivative;

        currentPwmL = constrain(pwm + correction, 0, 255);
        currentPwmR = constrain(pwm - correction, 0, 255);

        previousHeadingError = error;
      }

      previousPidTime = now;
    }
    else
    {
      currentPwmL = pwm;
      currentPwmR = pwm;
    }

    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  } 

  else if(_data == 2) 
  {
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);

    int pwm = applyStartRamp(230, _data);

    if(!headingHoldActive)
    {
      if(headingCaptureStart == 0)
        headingCaptureStart = millis();

      if(millis() - headingCaptureStart >= 200)
      {
        targetHeading = currentHeading;
        headingHoldActive = true;
        headingIntegral = 0;
        previousHeadingError = 0;
        previousPidTime = millis();
      }
    }

    if(headingHoldActive)
    {
      float error = normalizeHeadingError(targetHeading - currentHeading);

      unsigned long now = millis();

      float dt = (now - previousPidTime) / 1000.0f;

      if(dt > 0.001f)
      {
          float derivative = (error - previousHeadingError) / dt;

          float correction = Kp * error + Kd * derivative;

          currentPwmL = constrain(pwm - correction, 0, 255);
          currentPwmR = constrain(pwm + correction, 0, 255);

          previousHeadingError = error;
      }

      previousPidTime = now;
    }
    else
    {
      currentPwmL = pwm;
      currentPwmR = pwm;
    }

    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  }

  else if (_data >= 11 && _data <= 20)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);

    if(_data <= 13) {
      currentPwmL = 60;
      currentPwmR = 60;
    }
    else if(_data <= 16) {
      currentPwmL = 80;
      currentPwmR = 80;
    }
    else {
      currentPwmL = 120;
      currentPwmR = 120;
    }
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
}

  else if (_data >= 21 && _data <= 30) 
  {
      headingHoldActive = false;
      headingCaptureStart = 0;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = 0;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);

    if(_data <= 23) {
      currentPwmL = 80;
      currentPwmR = 80;
    }
    else if(_data <= 26) {
      currentPwmL = 120;
      currentPwmR = 120;
    }
    else {
      currentPwmL = 180;
      currentPwmR = 180;
    }

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if(_data >= 111 && _data <= 120) 
  {
      headingHoldActive = false;
      headingCaptureStart = 0;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = 0;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if(_data <= 113) {
      currentPwmL = 170;
      currentPwmR = 120;
    }

    else if(_data <= 116) {
      currentPwmL = 200;
      currentPwmR = 150;

    }
    else {
      currentPwmL = 240;
      currentPwmR = 80;
    }

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
    
  } 

  else if(_data >= 121 && _data <= 130)
  {
      headingHoldActive = false;
      headingCaptureStart = 0;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = 0;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if(_data <= 123) {
      currentPwmL = 120;
      currentPwmR = 170;
    }

    else if(_data <= 126) {
        currentPwmL = 150;
        currentPwmR = 200;
    }
      
    else {
      currentPwmL = 80;
      currentPwmR = 240;
    }
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if(_data >= 211 && _data <= 220) 
  {
      headingHoldActive = false;
      headingCaptureStart = 0;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = 0;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if(_data <= 213) {
      currentPwmL = 170;
      currentPwmR = 120;
    }

    else if(_data <= 216) {
      currentPwmL = 200;
      currentPwmR = 120;
    }
    else {
      currentPwmL = 240;
      currentPwmR = 80;
    }

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);


  } 
  else if(_data >= 221 && _data <= 230) 
  {
      headingHoldActive = false;
      headingCaptureStart = 0;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = 0;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if(_data <= 223) {
      currentPwmL = 120;
      currentPwmR = 170;
    }
    else if(_data <= 226) {
      currentPwmL = 120;
      currentPwmR = 200;
    }
    else {
      currentPwmL = 80;
      currentPwmR = 240;
    }

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  } 
  lastCommand = _data;
}

