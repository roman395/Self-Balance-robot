#include "pidTerminal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

PIDTerminal::PIDTerminal(Stream &serial, PIDController &controller)
    : serial(serial), controller(controller) {}

void PIDTerminal::begin()
{
    serial.println(F("PID commands: kp <value>, ki <value>, kd <value>, pid <p> <i> <d>, target <angle>, show, help"));
    show();
}

void PIDTerminal::show()
{
    PIDCoefficients gains = controller.getCoefficients();
    serial.print(F("Kp="));
    serial.print(gains.getKp(), 6);
    serial.print(F(" Ki="));
    serial.print(gains.getKi(), 6);
    serial.print(F(" Kd="));
    serial.print(gains.getKd(), 6);
    serial.print(F(" Target="));
    serial.println(controller.getSetpoint(), 6);
}

void PIDTerminal::update()
{
    // Limit work per loop so continuous input cannot starve sensor updates.
    for (uint8_t count = 0; count < 32 && serial.available(); ++count)
    {
        char ch = serial.read();
        if (ch == '\r' || ch == '\n')
        {
            if (overflow)
                serial.println(F("Error: command too long."));
            else if (length)
            {
                buffer[length] = '\0';
                execute();
            }
            length = 0;
            overflow = false;
        }
        else if (!overflow && (ch == '\b' || ch == 127))
        {
            if (length) --length;
        }
        else if (!overflow)
        {
            if (length < sizeof(buffer) - 1)
                buffer[length++] = ch;
            else
                overflow = true;
        }
    }
}

void PIDTerminal::execute()
{
    char *command = strtok(buffer, " \t");
    if (!command) return;

    if (!strcmp(command, "help") || !strcmp(command, "show"))
    {
        if (strtok(nullptr, " \t"))
        {
            serial.println(F("Error: unexpected argument."));
            return;
        }
        if (!strcmp(command, "help")) begin();
        else show();
        return;
    }

    bool all = !strcmp(command, "pid");
    bool p = !strcmp(command, "kp");
    bool i = !strcmp(command, "ki");
    bool d = !strcmp(command, "kd");
    bool target = !strcmp(command, "target");
    if (!all && !p && !i && !d && !target)
    {
        serial.println(F("Error: unknown command. Type help."));
        return;
    }

    float values[3];
    for (uint8_t index = 0; index < (all ? 3 : 1); ++index)
    {
        char *token = strtok(nullptr, " \t");
        char *end;
        if (!token)
        {
            serial.println(F("Error: missing value."));
            return;
        }
        values[index] = strtod(token, &end);
        if (end == token || *end || !isfinite(values[index]))
        {
            serial.println(F("Error: expected a finite number."));
            return;
        }
    }
    if (strtok(nullptr, " \t"))
    {
        serial.println(F("Error: too many values."));
        return;
    }

    if (target)
    {
        controller.setSetpoint(values[0]);
        show();
        return;
    }

    PIDCoefficients gains = controller.getCoefficients();
    controller.setCoefficients(all ? PIDCoefficients(values[0], values[1], values[2])
        : PIDCoefficients(p ? values[0] : gains.getKp(),
                          i ? values[0] : gains.getKi(),
                          d ? values[0] : gains.getKd()));
    show();
}
