#pragma once

#include <Arduino.h>

struct AutoCommand
{
    char type;     // F B L R S
    float value;   // metres or degrees
};

extern bool autonomousRunning;

bool autonomousLoadTrack(const char *trackStr);

void autonomousStart();

void autonomousStop();

void autonomousUpdate();

