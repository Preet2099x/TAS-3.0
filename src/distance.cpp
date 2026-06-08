#include <var.h>
#include <Arduino.h>

static long lastEncoderL = 0;
static long lastEncoderR = 0;

// Robot specifics
#define WHEEL_DIAMETER_M 0.15f
#define TICKS_PER_ENC_REV 4000.0f
#define MECH_RATIO 1.6f

// Pre-computed constant: cm per encoder tick
// = (PI * diameter_m * 100cm/m) / (ticks_per_rev * mech_ratio)
static const float CM_PER_TICK =
    (PI * WHEEL_DIAMETER_M * 100.0f) /
    (TICKS_PER_ENC_REV * MECH_RATIO);

static unsigned long lastDistanceTime = 0;

void distanceLoop()
{
    // Rate-limit to every 20ms — avoids rounding noise at
    // very small deltas while still being responsive
    unsigned long now = millis();
    if (now - lastDistanceTime < 20)
        return;
    lastDistanceTime = now;

    // Snapshot volatile encoder values with interrupts
    // disabled to prevent torn reads
    noInterrupts();
    long currentL = totalEncoder_L;
    long currentR = totalEncoder_R;
    interrupts();

    long deltaL = currentL - lastEncoderL;
    long deltaR = currentR - lastEncoderR;

    lastEncoderL = currentL;
    lastEncoderR = currentR;

    // Use abs() so distance accumulates regardless of
    // direction (odometer behaviour)
    float avgDelta = (abs(deltaL) + abs(deltaR)) * 0.5f;

    totalDistanceCm += avgDelta * CM_PER_TICK;
}