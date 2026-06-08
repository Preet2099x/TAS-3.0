#include <var.h>
#include <Arduino.h>

static unsigned long rampStartTime = 0;
static bool rampActive = false;
static int lastCommand = 0;
static int rampCommand = 0;

static int currentPwmL = 0;
static int currentPwmR = 0;

static unsigned long turnRampStartTime = 0;
static bool turnRampActive = false;
static int turnRampCommand = 0;

static unsigned long stopRampStartTime = 0;
static bool stopRampActive = false;

static int stopStartPwmL = 0;
static int stopStartPwmR = 0;

static bool headingHoldActive = false;
static unsigned long headingCaptureStart = 0;

float targetHeading = 0;

float headingIntegral = 0;
float previousHeadingError = 0;
float pidError = 0;
float pidCorrection = 0;
unsigned long previousPidTime = 0;

const float Kp = 2.0f;
const float Ki = 0.5f;
const float Kd = 0.05f;  // Low — BNO055 jitter amplifies through derivative

int debugPwmL = 0;
int debugPwmR = 0;

int applyStartRamp(int targetPWM, int currentCommand)
{
  if (rampActive && currentCommand == rampCommand)
  {
    unsigned long elapsed = millis() - rampStartTime;

    if (elapsed >= 1000)
    {
      rampActive = false;
      return targetPWM;
    }

    return 50 + ((targetPWM - 50) * elapsed) / 1000;
  }

  if (currentCommand != lastCommand && (currentCommand == 1 || currentCommand == 2))
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
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;

  return error;
}

int applyTurnRamp(int targetPWM, int currentCommand)
{
  if (turnRampActive && currentCommand == turnRampCommand)
  {
    unsigned long elapsed = millis() - turnRampStartTime;

      if (elapsed >= 50)
    {
      turnRampActive = false;
      return targetPWM;
    }

      return 50 + ((targetPWM - 50) * elapsed) / 50;
  }

  if (currentCommand != lastCommand)
  {
    turnRampStartTime = millis();
    turnRampCommand = currentCommand;
    turnRampActive = true;
    return 50;
  }

  return targetPWM;
}

void motion(int _data)
{
  if (_data == 0)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    if (!stopRampActive)
    {
      stopRampStartTime = millis();

      stopStartPwmL = currentPwmL;
      stopStartPwmR = currentPwmR;

      stopRampActive = true;
    }

    unsigned long elapsed = millis() - stopRampStartTime;

    if (elapsed >= 20) // 20ms Ramp Down
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
      turnRampActive = false;
      turnRampCommand = 0;

      lastCommand = 0;
      return;
    }

    float progress = elapsed / 20.0f; // 20ms Ramp Down
    float scale = 1.0f - (progress * progress);

    currentPwmL = stopStartPwmL * scale;
    currentPwmR = stopStartPwmR * scale;

    debugPwmL = 0;
    debugPwmR = 0;

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

    if (!headingHoldActive)
    {
      if (headingCaptureStart == 0)
        headingCaptureStart = millis();

      if (millis() - headingCaptureStart >= 200)
      {
        targetHeading = currentHeading;
        headingHoldActive = true;
        headingIntegral = 0;
        previousHeadingError = 0;
        previousPidTime = millis();
      }
    }

    if (headingHoldActive)
    {
      unsigned long now = millis();
      float dt = (now - previousPidTime) / 1000.0f;

      if (dt >= 0.01f) // Run PID at ~100Hz (every 10ms)
      {
        float error = normalizeHeadingError(targetHeading - currentHeading);

        float derivative = (error - previousHeadingError) / dt;

        headingIntegral += error * dt;
        headingIntegral = constrain(headingIntegral, -15.0f, 15.0f);
        float correction = Kp * error + Ki * headingIntegral + Kd * derivative;
        pidError = error;
        pidCorrection = correction;

        currentPwmL = constrain(pwm + correction, 0, 255);
        currentPwmR = constrain(pwm - correction, 0, 255);

        debugPwmL = currentPwmL;
        debugPwmR = currentPwmR;

        previousHeadingError = error;
        previousPidTime = now; // Only reset when PID actually ran
      }
    }
    else
    {
      currentPwmL = pwm;
      currentPwmR = pwm;

      debugPwmL = currentPwmL;
      debugPwmR = currentPwmR;
    }

    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  }

  else if (_data == 2)
  {
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);

    int pwm = applyStartRamp(230, _data);

    if (!headingHoldActive)
    {
      if (headingCaptureStart == 0)
        headingCaptureStart = millis();

      if (millis() - headingCaptureStart >= 200)
      {
        targetHeading = currentHeading;
        headingHoldActive = true;
        headingIntegral = 0;
        previousHeadingError = 0;
        previousPidTime = millis();
      }
    }

    if (headingHoldActive)
    {
      unsigned long now = millis();
      float dt = (now - previousPidTime) / 1000.0f;

      if (dt >= 0.01f) // Run PID at ~100Hz (every 10ms)
      {
        float error = normalizeHeadingError(targetHeading - currentHeading);

        float derivative = (error - previousHeadingError) / dt;

        headingIntegral += error * dt;
        headingIntegral = constrain(headingIntegral, -15.0f, 15.0f);
        float correction = Kp * error + Ki * headingIntegral + Kd * derivative;
        pidError = error;
        pidCorrection = correction;

        currentPwmL = constrain(pwm - correction, 0, 255);
        currentPwmR = constrain(pwm + correction, 0, 255);

        debugPwmL = currentPwmL;
        debugPwmR = currentPwmR;

        previousHeadingError = error;
        previousPidTime = now; // Only reset when PID actually ran
      }
    }
    else
    {
      currentPwmL = pwm;
      currentPwmR = pwm;

      debugPwmL = currentPwmL;
      debugPwmR = currentPwmR;
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
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 13)
    {
      currentPwmL = 60;
      currentPwmR = 60;
    }
    else if (_data <= 16)
    {
      currentPwmL = 80;
      currentPwmR = 80;
    }
    else
    {
      currentPwmL = 120;
      currentPwmR = 120;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
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
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 23)
    {
      currentPwmL = 80;
      currentPwmR = 80;
    }
    else if (_data <= 26)
    {
      currentPwmL = 120;
      currentPwmR = 120;
    }
    else
    {
      currentPwmL = 180;
      currentPwmR = 180;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 111 && _data <= 120)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 113)
    {
      currentPwmL = 170;
      currentPwmR = 120;
      // currentPwmL = applyTurnRamp(170, _data);
      // currentPwmR = applyTurnRamp(120, _data);
    }

    else if (_data <= 116)
    {
      currentPwmL = 200;
      currentPwmR = 120;
      // currentPwmL = applyTurnRamp(200, _data);
      // currentPwmR = applyTurnRamp(150, _data);
    }
    else
    {
      currentPwmL = 240;
      currentPwmR = 80;
      // currentPwmL = applyTurnRamp(240, _data);
      // currentPwmR = applyTurnRamp(80, _data);
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 121 && _data <= 130)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 123)
    {
      currentPwmL = 120;
      currentPwmR = 170;
      // currentPwmL = applyTurnRamp(120, _data);
      // currentPwmR = applyTurnRamp(170, _data);
    }

    else if (_data <= 126)
    {
      currentPwmL = 120;
      currentPwmR = 200;
      // currentPwmL = applyTurnRamp(120, _data);
      // currentPwmR = applyTurnRamp(200, _data);
    }

    else
    {
      currentPwmL = 80;
      currentPwmR = 240;
      // currentPwmL = applyTurnRamp(80, _data);
      // currentPwmR = applyTurnRamp(240, _data);
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 211 && _data <= 220)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 213)
    {
      currentPwmL = 170;
      currentPwmR = 120;
      // currentPwmL = applyTurnRamp(170, _data);
      // currentPwmR = applyTurnRamp(120, _data);
    }

    else if (_data <= 216)
    {
      currentPwmL = 200;
      currentPwmR = 120;
      // currentPwmL = applyTurnRamp(200, _data);
      // currentPwmR = applyTurnRamp(120, _data);
    }
    else
    {
      currentPwmL = 240;
      currentPwmR = 80;
      // currentPwmL = applyTurnRamp(240, _data);
      // currentPwmR = applyTurnRamp(80, _data);
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }
  else if (_data >= 221 && _data <= 230)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 223)
    {
      currentPwmL = 120;
      currentPwmR = 170;
      // currentPwmL = applyTurnRamp(120, _data);
      // currentPwmR = applyTurnRamp(170, _data);
    }
    else if (_data <= 226)
    {
      currentPwmL = 120;
      currentPwmR = 200;
      // currentPwmL = applyTurnRamp(120, _data);
      // currentPwmR = applyTurnRamp(200, _data);
    }
    else
    {
      currentPwmL = 80;
      currentPwmR = 240;
      // currentPwmL = applyTurnRamp(80, _data);
      // currentPwmR = applyTurnRamp(240, _data);
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }
  lastCommand = _data;
}
