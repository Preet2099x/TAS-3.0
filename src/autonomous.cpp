#include "autonomous.h"
#include <var.h>

#define MAX_COMMANDS 30

AutoCommand track[MAX_COMMANDS];

int trackLength = 0;
int currentStep = 0;

bool autonomousRunning = false;
bool autonomousPaused = false;

static float pauseRemainingValue = 0;
static float resumeTargetValue = 0;

static bool stepInitialized = false;

static float stepStartDistance = 0;

static float accumulatedAngle = 0;
static float previousHeading = 0;
static const float TURN_TOLERANCE = 2.0f;

float normalizeAutoAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

void nextStep()
{
    data = 0;

    currentStep++;
    stepInitialized = false;

    if (currentStep >= trackLength)
    {
        autonomousRunning = false;
        data = 0;

        Serial.println("AUTO COMPLETE");
    }
}

bool autonomousLoadTrack(const char *trackStr)
{
    trackLength = 0;

    char buffer[512];

    strncpy(buffer, trackStr, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, ",");

    while (token != nullptr && trackLength < MAX_COMMANDS)
    {
        track[trackLength].type = toupper(token[0]);

        if (track[trackLength].type == 'S')
            track[trackLength].value = 0;
        else
            track[trackLength].value = atof(token + 1);

        trackLength++;

        token = strtok(nullptr, ",");
    }

    return trackLength > 0;
}

void autonomousStart()
{
    currentStep = 0;
    stepInitialized = false;
    autonomousRunning = true;
    autonomousPaused = false;
    pauseRemainingValue = 0;
    resumeTargetValue = 0;

    Serial.println("AUTO START");
}

void autonomousStop()
{
    autonomousRunning = false;
    autonomousPaused = false;
    data = 0;

    Serial.println("AUTO STOP");
}

void autonomousPause()
{
    if (!autonomousRunning)
        return;

    autonomousPaused = !autonomousPaused;

    if (autonomousPaused)
    {
        // --- Capture progress so we can resume later ---
        if (currentStep < trackLength && stepInitialized)
        {
            AutoCommand &cmd = track[currentStep];

            if (cmd.type == 'F' || cmd.type == 'B')
            {
                float effectiveTarget = (resumeTargetValue > 0) ? resumeTargetValue : cmd.value;
                float travelled =
                    (totalDistanceCm - stepStartDistance) / 100.0f;
                pauseRemainingValue = effectiveTarget - travelled;

                if (pauseRemainingValue < 0)
                    pauseRemainingValue = 0;

                Serial.print("PAUSED — remaining ");
                Serial.print(pauseRemainingValue);
                Serial.println("m");
            }
            else if (cmd.type == 'L' || cmd.type == 'R')
            {
                float effectiveTarget = (resumeTargetValue > 0) ? resumeTargetValue : cmd.value;
                pauseRemainingValue = effectiveTarget - accumulatedAngle;

                if (pauseRemainingValue < 0)
                    pauseRemainingValue = 0;

                Serial.print("PAUSED — remaining ");
                Serial.print(pauseRemainingValue);
                Serial.println(" deg");
            }
            else
            {
                pauseRemainingValue = 0;
                Serial.println("PAUSED");
            }
        }
        else
        {
            pauseRemainingValue = 0;
            Serial.println("PAUSED");
        }

        data = 0; // stop motors
    }
    else
    {
        // --- Resume: re-snapshot starting point, use remaining value ---
        if (currentStep < trackLength && stepInitialized)
        {
            AutoCommand &cmd = track[currentStep];

            if (cmd.type == 'F' || cmd.type == 'B')
            {
                stepStartDistance = totalDistanceCm;
                resumeTargetValue = pauseRemainingValue;

                Serial.print("RESUMED — ");
                Serial.print(pauseRemainingValue);
                Serial.println("m left");
            }
            else if (cmd.type == 'L' || cmd.type == 'R')
            {
                accumulatedAngle = 0;
                previousHeading = currentHeading;
                resumeTargetValue = pauseRemainingValue;

                Serial.print("RESUMED — ");
                Serial.print(pauseRemainingValue);
                Serial.println(" deg left");
            }
            else
            {
                Serial.println("RESUMED");
            }
        }
        else
        {
            Serial.println("RESUMED");
        }
    }
}

void autonomousUpdate()
{
    if (!autonomousRunning || autonomousPaused)
        return;

    if (currentStep >= trackLength)
    {
        autonomousStop();
        return;
    }

    AutoCommand &cmd = track[currentStep];

    switch (cmd.type)
    {
    case 'F':
    {
        if (!stepInitialized)
        {
            stepStartDistance = totalDistanceCm;
            stepInitialized = true;

            Serial.print("FWD START ");
            Serial.print(cmd.value);
            Serial.println("m");
        }

        data = 1;

        float target = (resumeTargetValue > 0) ? resumeTargetValue : cmd.value;
        float travelled =
            (totalDistanceCm - stepStartDistance) / 100.0f;

        if (travelled >= target)
        {
            Serial.print("FWD DONE ");
            Serial.println(travelled);

            resumeTargetValue = 0;
            nextStep();
        }

        break;
    }

    case 'B':
    {
        if (!stepInitialized)
        {
            stepStartDistance = totalDistanceCm;
            stepInitialized = true;

            Serial.print("BACK START ");
            Serial.print(cmd.value);
            Serial.println("m");
        }

        data = 2;

        float target = (resumeTargetValue > 0) ? resumeTargetValue : cmd.value;
        float travelled =
            (totalDistanceCm - stepStartDistance) / 100.0f;

        if (travelled >= target)
        {
            Serial.print("BACK DONE ");
            Serial.println(travelled);

            resumeTargetValue = 0;
            nextStep();
        }

        break;
    }
    case 'L':
    {
        if (!stepInitialized)
        {
            accumulatedAngle = 0;
            previousHeading = currentHeading;
            resumeTargetValue = 0;
            stepInitialized = true;

            Serial.print("LEFT START ");
            Serial.print(cmd.value);
            Serial.println(" deg");
        }

        data = 25; // left turn

        // Only process heading delta when the IMU has new data
        if (headingUpdated)
        {
            headingUpdated = false;

            float delta =
                normalizeAutoAngle(currentHeading - previousHeading);

            // Left turn: only accumulate negative rotation (heading decreasing)
            if (delta < 0)
                accumulatedAngle += (-delta); // store as positive magnitude
            previousHeading = currentHeading;
        }

        float target = (resumeTargetValue > 0) ? resumeTargetValue : cmd.value;

        if (accumulatedAngle >= (target - TURN_TOLERANCE))
        {
            Serial.print("LEFT DONE ");
            Serial.println(accumulatedAngle);

            resumeTargetValue = 0;
            nextStep();
        }

        break;
    }

    case 'R':
    {
        if (!stepInitialized)
        {
            accumulatedAngle = 0;
            previousHeading = currentHeading;
            resumeTargetValue = 0;
            stepInitialized = true;

            Serial.print("RIGHT START ");
            Serial.print(cmd.value);
            Serial.println(" deg");
        }

        data = 15; // right turn

        // Only process heading delta when the IMU has new data
        if (headingUpdated)
        {
            headingUpdated = false;

            float delta =
                normalizeAutoAngle(currentHeading - previousHeading);

            // Right turn: only accumulate positive rotation (heading increasing)
            if (delta > 0)
                accumulatedAngle += delta;
            previousHeading = currentHeading;
        }

        float target = (resumeTargetValue > 0) ? resumeTargetValue : cmd.value;

        if (accumulatedAngle >= (target - TURN_TOLERANCE))
        {
            Serial.print("RIGHT DONE ");
            Serial.println(accumulatedAngle);

            resumeTargetValue = 0;
            nextStep();
        }

        break;
    }

    case 'S':
    {
        Serial.println("STOP");
        autonomousStop();
        break;
    }
    }
}