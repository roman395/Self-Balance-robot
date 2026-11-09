#pragma once

#include <Arduino.h>
#include "pid.h"

// Call update() regularly. Commands end with CR, LF, or CRLF.
class PIDTerminal
{
public:
    PIDTerminal(Stream &serial, PIDController &controller);
    void begin();
    void update();

private:
    Stream &serial;
    PIDController &controller;
    char buffer[80];
    uint8_t length = 0;
    bool overflow = false;

    void execute();
    void show();
};
