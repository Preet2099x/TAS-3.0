#include <var.h>
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

bool frontBlocked = false;
bool rearBlocked = false;
bool leftBlocked = false;
bool rightBlocked = false;

static char cameraBuf[256];
static int cameraBufIdx = 0;

const float LONG_THRESHOLD = 70.0f;  // f1,b1,l1,r1
const float SHORT_THRESHOLD = 30.0f; // f2,b2,l2,r2

static unsigned long lastCameraPacketTime = 0;

bool commandBlocked(int cmd)
{
    if (frontBlocked)
    {
        if (cmd == 1 ||
            (cmd >= 111 && cmd <= 120) ||
            (cmd >= 121 && cmd <= 130))
        {
            return true;
        }
    }

    if (rearBlocked)
    {
        if (cmd == 2 ||
            (cmd >= 211 && cmd <= 220) ||
            (cmd >= 221 && cmd <= 230))
        {
            return true;
        }
    }

    if (leftBlocked)
    {
        if ((cmd >= 11 && cmd <= 20) ||
            (cmd >= 111 && cmd <= 120) ||
            (cmd >= 211 && cmd <= 220))
        {
            return true;
        }
    }

    if (rightBlocked)
    {
        if ((cmd >= 21 && cmd <= 30) ||
            (cmd >= 121 && cmd <= 130) ||
            (cmd >= 221 && cmd <= 230))
        {
            return true;
        }
    }

    return false;
}


static float extractValue(const char *buf, const char *key)
{
    char *p = strstr((char *)buf, key);

    if (p == nullptr)
        return -1;

    p += strlen(key);

    return atof(p);
}

void cameraHandleByte(char c)
{
    if (c == '\r')
        return;

    if (c != '\n')
    {
        if (cameraBufIdx < (int)sizeof(cameraBuf) - 1)
        {
            cameraBuf[cameraBufIdx++] = c;
        }

        return;
    }

    cameraBuf[cameraBufIdx] = '\0';
    cameraBufIdx = 0;

    if (strncmp(cameraBuf, "[camera]", 8) != 0)
    {
        return;
    }

    if (strncmp(cameraBuf, "[camera]", 8) != 0)
    {
        return;
    }

    lastCameraPacketTime = millis();

    float f1 = extractValue(cameraBuf, "f1-dist=");
    float f2 = extractValue(cameraBuf, "f2-dist=");

    float b1 = extractValue(cameraBuf, "b1-dist=");
    float b2 = extractValue(cameraBuf, "b2-dist=");

    float l1 = extractValue(cameraBuf, "l1-dist=");
    float l2 = extractValue(cameraBuf, "l2-dist=");

    float r1 = extractValue(cameraBuf, "r1-dist=");
    float r2 = extractValue(cameraBuf, "r2-dist=");

    frontBlocked =
        (f1 >= 0 && f1 <= LONG_THRESHOLD) ||
        (f2 >= 0 && f2 <= SHORT_THRESHOLD);

    rearBlocked =
        (b1 >= 0 && b1 <= LONG_THRESHOLD) ||
        (b2 >= 0 && b2 <= SHORT_THRESHOLD);

    leftBlocked =
        (l1 >= 0 && l1 <= LONG_THRESHOLD) ||
        (l2 >= 0 && l2 <= SHORT_THRESHOLD);

    rightBlocked =
        (r1 >= 0 && r1 <= LONG_THRESHOLD) ||
        (r2 >= 0 && r2 <= SHORT_THRESHOLD);

    if (!cameraEnabled)
    {
        return;
    }

    if (commandBlocked(data))
    {
        data = 0;
    }

    /*
    Serial.print("CAM F:");
    Serial.print(frontBlocked);

    Serial.print(" B:");
    Serial.print(rearBlocked);

    Serial.print(" L:");
    Serial.print(leftBlocked);

    Serial.print(" R:");
    Serial.println(rightBlocked);
    */
}