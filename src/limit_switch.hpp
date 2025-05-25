#ifndef RUSHBOTT_HARDWARE_LIMIT_SWITCH_HPP
#define RUSHBOTT_HARDWARE_LIMIT_SWITCH_HPP
#include <Arduino.h>

struct LimitSwitch {
    int pin;
    int stepper_index;
    int limit;
    bool last_state = LOW;           
    bool pressed = false;
    unsigned long last_debounce = 0;

    LimitSwitch(int _pin, int _stepper_index, int _limit)
      : pin(_pin)
      , stepper_index(_stepper_index)
      , limit(_limit)
      , last_state(HIGH)
      , pressed(false)
      , last_debounce(0)
    {}
};

#endif // RUSHBOTT_HARDWARE_LIMIT_SWITCH_HPP