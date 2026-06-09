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

float targetHeading = 0;

float headingIntegral = 0;
float previousHeadingError = 0;
float pidError = 0;
float pidCorrection = 0;
unsigned long previousPidTime = 0;

const float Kp = 2.0f;
const float Ki = 0.5f;
const float Kd = 0.05f; // Low — BNO055 jitter amplifies through derivative

// Speed PID (controls average RPM → basePWM)
static const float TARGET_RPM = 120.0f;
static const float Kp_speed = 1.5f;
static const float Ki_speed = 0.3f;
static const float Kd_speed = 0.01f;

static float speedIntegral = 0;
static float previousSpeedError = 0;
static unsigned long previousSpeedPidTime = 0;
static float currentBasePWM = 0; // Output of speed PID, persists between calls

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

  if (lastCommand == 0 &&
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
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;

  return error;
}

// Speed PID: single controller on average RPM → basePWM
// Only runs when rpmPidEnabled == true; otherwise returns fixed PWM.
int computeBasePWM(int rampedPWM)
{
  if (!rpmPidEnabled)
    return rampedPWM; // Legacy: fixed PWM from ramp

  unsigned long now = millis();
  float dt = (now - previousSpeedPidTime) / 1000.0f;

  // Only run at ~10Hz (every 100ms) — matches RPM measurement update rate
  if (dt < 0.08f)
    return (int)currentBasePWM; // Return last computed value

  previousSpeedPidTime = now;

  float error = TARGET_RPM - measuredAvgRPM;

  speedIntegral += error * dt;
  speedIntegral = constrain(speedIntegral, -50.0f, 50.0f);

  float derivative = (error - previousSpeedError) / dt;
  previousSpeedError = error;

  float output = Kp_speed * error + Ki_speed * speedIntegral + Kd_speed * derivative;

  // Adjust basePWM incrementally from current value
  currentBasePWM += output * dt; // Smooth integration
  currentBasePWM = constrain(currentBasePWM, 50.0f, 255.0f);

  return (int)currentBasePWM;
}

void motion(int _data)
{
  if (_data == 0)
  {
    headingHoldActive = false;
    headingIntegral = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    // Reset speed PID state
    speedIntegral = 0;
    previousSpeedError = 0;
    previousSpeedPidTime = 0;
    currentBasePWM = 0;

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

    int rampPwm = applyStartRamp(230, _data);
    int pwm;

    // During ramp-up, use fixed ramp; after ramp, switch to speed PID
    if (rampActive)
    {
      pwm = rampPwm;
      // Seed the speed PID output so it doesn't jump when ramp ends
      currentBasePWM = rampPwm;
      previousSpeedPidTime = millis();
    }
    else
    {
      pwm = computeBasePWM(rampPwm);
    }

    if (!headingHoldActive)
    {
      targetHeading = currentHeading;

      headingHoldActive = true;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = millis();
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

    int rampPwm = applyStartRamp(230, _data);
    int pwm;

    // During ramp-up, use fixed ramp; after ramp, switch to speed PID
    if (rampActive)
    {
      pwm = rampPwm;
      currentBasePWM = rampPwm;
      previousSpeedPidTime = millis();
    }
    else
    {
      pwm = computeBasePWM(rampPwm);
    }

    if (!headingHoldActive)
    {
      targetHeading = currentHeading;

      headingHoldActive = true;
      headingIntegral = 0;
      previousHeadingError = 0;
      previousPidTime = millis();
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
    }

    else if (_data <= 116)
    {
      currentPwmL = 200;
      currentPwmR = 150;
    }
    else
    {
      currentPwmL = 240;
      currentPwmR = 80;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 121 && _data <= 130)
  {
    headingHoldActive = false;
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
    }

    else if (_data <= 126)
    {
      currentPwmL = 150;
      currentPwmR = 200;
    }

    else
    {
      currentPwmL = 80;
      currentPwmR = 240;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 211 && _data <= 220)
  {
    headingHoldActive = false;
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
    }

    else if (_data <= 216)
    {
      currentPwmL = 200;
      currentPwmR = 120;
    }
    else
    {
      currentPwmL = 240;
      currentPwmR = 80;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }
  else if (_data >= 221 && _data <= 230)
  {
    headingHoldActive = false;
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
    }
    else if (_data <= 226)
    {
      currentPwmL = 120;
      currentPwmR = 200;
    }
    else
    {
      currentPwmL = 80;
      currentPwmR = 240;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }
  lastCommand = _data;
}
