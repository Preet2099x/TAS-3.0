#pragma once

#include <Arduino.h>

struct AutoCommand
{
    char type;     // F B L R S
    float value;   // metres or degrees
};

extern bool autonomousRunning;
extern bool autonomousPaused;

bool autonomousLoadTrack(const char *trackStr);

void autonomousStart();

void autonomousStop();

void autonomousPause();

void autonomousUpdate();

