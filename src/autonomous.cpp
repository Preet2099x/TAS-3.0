#include "autonomous.h"
#include <var.h>

#define MAX_COMMANDS 50

AutoCommand track[MAX_COMMANDS];

int trackLength = 0;
int currentStep = 0;

bool autonomousRunning = false;
bool autonomousPaused = false;

static float pauseRemainingValue = 0;

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
                float travelled =
                    (totalDistanceCm - stepStartDistance) / 100.0f;
                pauseRemainingValue = cmd.value - travelled;

                if (pauseRemainingValue < 0)
                    pauseRemainingValue = 0;

                Serial.print("PAUSED — remaining ");
                Serial.print(pauseRemainingValue);
                Serial.println("m");
            }
            else if (cmd.type == 'L' || cmd.type == 'R')
            {
                float turned = fabs(accumulatedAngle);
                pauseRemainingValue = cmd.value - turned;

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
                cmd.value = pauseRemainingValue;

                Serial.print("RESUMED — ");
                Serial.print(pauseRemainingValue);
                Serial.println("m left");
            }
            else if (cmd.type == 'L' || cmd.type == 'R')
            {
                accumulatedAngle = 0;
                previousHeading = currentHeading;
                cmd.value = pauseRemainingValue;

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

        float travelled =
            (totalDistanceCm - stepStartDistance) / 100.0f;

        if (travelled >= cmd.value)
        {
            Serial.print("FWD DONE ");
            Serial.println(travelled);

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

        float travelled =
            (totalDistanceCm - stepStartDistance) / 100.0f;

        if (travelled >= cmd.value)
        {
            Serial.print("BACK DONE ");
            Serial.println(travelled);

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
            stepInitialized = true;

            Serial.print("LEFT START ");
            Serial.print(cmd.value);
            Serial.println(" deg");
        }

        data = 21; // left turn

        float delta =
            normalizeAutoAngle(currentHeading - previousHeading);

        accumulatedAngle += delta;
        previousHeading = currentHeading;

        float turned = fabs(accumulatedAngle);

        if (turned >= (cmd.value - TURN_TOLERANCE))
        {
            Serial.print("LEFT DONE ");
            Serial.println(turned);

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
            stepInitialized = true;

            Serial.print("RIGHT START ");
            Serial.print(cmd.value);
            Serial.println(" deg");
        }

        data = 11; // right turn

        float delta =
            normalizeAutoAngle(currentHeading - previousHeading);

        accumulatedAngle += delta;
        previousHeading = currentHeading;

        float turned = fabs(accumulatedAngle);

        if (turned >= (cmd.value - TURN_TOLERANCE))
        {
            Serial.print("RIGHT DONE ");
            Serial.println(turned);

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