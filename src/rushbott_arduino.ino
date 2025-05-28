#include "AccelStepper.h"
#include "motor_packet.hpp"
#include "limit_switch.hpp"
#include <Arduino.h>

#define LED_PIN 13

#define LOWER_ARM_LOW_LIMIT_SWITCH_PIN 3
#define LOWER_ARM_HIGH_LIMIT_SWITCH_PIN 5
#define LOWER_ARM_DIR_PIN 7
#define LOWER_ARM_STEP_PIN 8

#define UPPER_ARM_HIGH_LIMIT_SWITCH_PIN 6
#define UPPER_ARM_DIR_PIN 9
#define UPPER_ARM_STEP_PIN 10

#define MOTOR_INTERFACE_TYPE 1

const long steps_per_rev = 6400;
const double steps_per_degree = steps_per_rev / 360.0;
const double steps_per_rad = steps_per_rev / (2*M_PI);

AccelStepper steppers[] = {
  AccelStepper(MOTOR_INTERFACE_TYPE, LOWER_ARM_STEP_PIN, LOWER_ARM_DIR_PIN),
  AccelStepper(MOTOR_INTERFACE_TYPE, UPPER_ARM_STEP_PIN, UPPER_ARM_DIR_PIN)
};

const int num_steppers = sizeof(steppers) / sizeof(steppers[0]);

LimitSwitch switches[] = {
  {LOWER_ARM_LOW_LIMIT_SWITCH_PIN, 0, (int)(155*5.785714*(steps_per_degree))},
  {LOWER_ARM_HIGH_LIMIT_SWITCH_PIN, 0, (int)(90*(steps_per_degree))},
  {UPPER_ARM_HIGH_LIMIT_SWITCH_PIN, 1, (int)(240*5.785714*(steps_per_degree))}
};

const int num_switches = sizeof(switches) / sizeof(switches[0]);
const unsigned long debounce_delay = 25;
int arm_state = -1;
bool calibrating = false;
const double calibration_speed = 0.3*steps_per_rev;

const int packet_size = sizeof(MotorPacket);
uint8_t packet_buffer[packet_size];
MotorPacket packet = {};
size_t byte_count = 0;
unsigned long packet_start_time = 0;
const unsigned int timeout_ms = 6;

unsigned long last_packet_start_time_us = 0;

unsigned long last_packet_total_time_us = 0;

void setup() {
  for (int i = 0; i < num_switches; i++)
  {
    pinMode(switches[i].pin, INPUT_PULLUP);
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int i = 0; i < num_steppers; i++)
  {
    steppers[i].setMaxSpeed(steps_per_rev * 6);
    steppers[i].setAcceleration(steps_per_rev * 3);
  }

  steppers[1].setPinsInverted(true,false,false);

  Serial.begin(115200);
}

void loop() {
  
  if (Serial.available() > 0)
  {      
    uint8_t byte = Serial.read();
    
    if (byte_count == 0 && byte == 0x64)
    {
      packet_start_time = millis();
    }
    if (packet_start_time > 0 && byte_count < packet_size)
    {
      packet_buffer[byte_count] = byte;
      byte_count++;
    }
    if (byte_count == packet_size)
    {
      memcpy(&packet, packet_buffer, packet_size);

      handle_packet(packet);
      
      memcpy(packet_buffer, &packet, packet_size);
      Serial.write(packet_buffer, packet_size);

      byte_count = 0;
      packet_start_time = 0;
    }
  }

  if (byte_count > 0 && millis() - packet_start_time > timeout_ms) 
  {
    byte_count = 0;
    packet_buffer[0] = 0x00;
    packet_start_time = 0;
  }

  // poll_switches();

  // if (arm_state == -1)
  // {
  //   digitalWrite(LED_PIN, LOW);
  //   for (int i = 0; i < num_steppers; i++)
  //   {
  //     steppers[i].run();
  //   }
  // }
  // else if (arm_state < num_switches)
  // {
  //   digitalWrite(LED_PIN, HIGH);
  //   calibrate();
  // }
  // else if (arm_state == num_switches + 1)
  // {
  //   digitalWrite(LED_PIN, HIGH);
  //   initialize();

  // }
  
}

void poll_switches()
{
  for (int i = 0; i < num_switches; i++)
  {
    LimitSwitch &sw = switches[i];
    bool current_state = digitalRead(sw.pin);

    if (current_state != sw.last_state)
    {
      sw.last_debounce = millis();
    }

    if ((millis() - sw.last_debounce) > debounce_delay)
    {
      if (current_state == HIGH && !sw.pressed)
      {
        sw.pressed = true;
        steppers[sw.stepper_index].setCurrentPosition(sw.limit);
      }
      else if (current_state == LOW && sw.pressed)
      {
        sw.pressed = false;
      }
    }

    sw.last_state = current_state;
  }
}

void calibrate()
{
  int direction = 0;
  LimitSwitch &lim_switch = switches[arm_state];
  AccelStepper &stepper = steppers[lim_switch.stepper_index];

  if (arm_state == 1)
  {
    arm_state++;
    return;
  }

  if (!calibrating)
  {
    if (lim_switch.pressed)
    {
    stepper.move(-0.1*5.785714*steps_per_rev);
    stepper.setSpeed(-calibration_speed);
    }
    else if (stepper.distanceToGo() == 0)
    {
      calibrating = true;
      stepper.move(5.785714*steps_per_rev);
      stepper.setSpeed(calibration_speed);
    }
  }
  else if (lim_switch.pressed)
  {
    stepper.stop();
    calibrating = false;
    arm_state++;
  }
  stepper.runSpeedToPosition();
}

void initialize()
{
  bool initialized = true;

  int index = 0;

  for (auto& stepper : steppers)
  {
    long distance = stepper.distanceToGo();

    if (distance != 0) {
      initialized = false;
      stepper.setSpeed(calibration_speed*(distance > 0 ? -1.0 : 1.0));
    }
    stepper.runSpeedToPosition();
    index++;  }

  if (initialized)
  {
    arm_state++;
  }
}

void handle_packet(MotorPacket &packet)
{
  if (packet.checksum != packet.calculate_checksum())
  {
    packet.flag = MotorPacket::NACK;
  }
  else if (packet.flag == MotorPacket::MOT)
  {
    packet.flag = MotorPacket::ACK;
    // for (int i = 0; i < num_steppers; i++)
    // {
    //   steppers[i].moveTo(packet.stepper[i]);
    // }
  }
  else if (packet.flag == MotorPacket::ENC)
  {
    // for (int i = 0; i < num_steppers; i++)
    // {
    //   packet.stepper[i] = steppers[i].currentPosition();
    // }
  }
  else if (packet.flag == MotorPacket::CAL)
  {
    packet.flag = MotorPacket::ACK;
    
    // // check if system is in active state
    // if (arm_state == -1)
    // {
    //   arm_state = 0;
    // }
    // // check if system has completed calibration -> set stepper initialization positions
    // else if (arm_state == num_switches)
    // {
    //   for (int i = 0; i < num_steppers; i++)
    //   {
    //     steppers[i].moveTo(packet.stepper[i]);
    //   }
    //   arm_state++;
    // }
    // // check if system has completed initialization
    // else if (arm_state == num_switches + 2)
    // {
    //   arm_state = -1;
    //   packet.flag = MotorPacket::CAL;
    // }
  }
  else if (packet.flag == MotorPacket::HEY)
  {
    packet.flag = MotorPacket::ACK;
  }
  else
  {
    packet.flag = MotorPacket::NACK;
  }
  packet.checksum = packet.calculate_checksum();
}