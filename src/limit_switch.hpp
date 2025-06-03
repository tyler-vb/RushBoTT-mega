#ifndef RUSHBOTT_HARDWARE_LIMIT_SWITCH_HPP
#define RUSHBOTT_HARDWARE_LIMIT_SWITCH_HPP
#include <Arduino.h>

struct LimitSwitch {
    int pin;
    int stepper_index;
    int limit;        
};

#endif // RUSHBOTT_HARDWARE_LIMIT_SWITCH_HPP