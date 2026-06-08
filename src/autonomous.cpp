#include "autonomous.h"
#include <var.h>

#define MAX_COMMANDS 50

AutoCommand track[MAX_COMMANDS];

int trackLength = 0;
int currentStep = 0;

bool autonomousRunning = false;

static bool stepInitialized = false;

static float stepStartDistance = 0;

static float accumulatedAngle = 0;
static float previousHeading = 0;

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

    Serial.println("AUTO START");
}

void autonomousStop()
{
    autonomousRunning = false;
    data = 0;

    Serial.println("AUTO STOP");
}

void autonomousUpdate()
{
    if (!autonomousRunning)
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
        break;

    case 'L':
        break;

    case 'R':
        break;

    case 'S':
        autonomousStop();
        break;
    }
}