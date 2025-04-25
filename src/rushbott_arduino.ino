#include "AccelStepper.h"
#include "motor_packet.hpp"


#define LED_PIN 13

#define LOWER_LIMIT_SWITCH_PIN 2
#define LOWER_DIR_PIN 8
#define LOWER_STEP_PIN 9

#define UPPER_LIMIT_SWITCH_PIN 3
#define UPPER_DIR_PIN 6
#define UPPER_STEP_PIN 7

#define MOTOR_INTERFACE_TYPE 1

AccelStepper lower_arm_stepper(MOTOR_INTERFACE_TYPE, LOWER_STEP_PIN, LOWER_DIR_PIN);
AccelStepper upper_arm_stepper(MOTOR_INTERFACE_TYPE, UPPER_STEP_PIN, UPPER_DIR_PIN);
AccelStepper* steppers[] = {&lower_arm_stepper, &upper_arm_stepper};

size_t num_steppers = sizeof(steppers) / sizeof(steppers[0]);

const int step_per_rev = 1600;
const double step_per_degree = step_per_rev / 360.0;
const double step_per_rad = step_per_rev / (2*M_PI);

const int stepper_lower_limits[] = {10 * (step_per_degree), -360 * (step_per_degree)};
const int stepper_upper_limits[] = {360 * (step_per_degree), 0 * (step_per_degree)};

const int packet_size = sizeof(MotorPacket);
uint8_t packet_buffer[packet_size];
size_t byte_count = 0;
unsigned long packet_start_time = 0;
const unsigned long timeout_ms = 200;

void setup() {
  pinMode(LOWER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(UPPER_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LOWER_LIMIT_SWITCH_PIN), calibrate_lower_arm_stepper, RISING);
  attachInterrupt(digitalPinToInterrupt(UPPER_LIMIT_SWITCH_PIN), calibrate_upper_arm_stepper, RISING);

  for (auto i = 0u; i < num_steppers; i++)
  {
    steppers[i]->setMaxSpeed(step_per_rev * 6);
    steppers[i]->setAcceleration(step_per_rev * 3);
    steppers[i]->setPinsInverted(true,false,false);
  }

  calibrate_lower_arm_stepper();
  calibrate_upper_arm_stepper();

  Serial.begin(9600);
}

void loop() {

  // Serial event loop
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
      MotorPacket packet = {};
      memcpy(&packet, packet_buffer, packet_size);

      handle_packet(packet);

      memcpy(packet_buffer, &packet, packet_size);
      Serial.write(packet_buffer, packet_size);

      byte_count = 0;
      packet_start_time = 0;
      digitalWrite(LED_PIN, LOW);
    }
  }

  if (byte_count > 0 && millis() - packet_start_time > timeout_ms) 
  {
    byte_count = 0;
    packet_start_time = 0;
    digitalWrite(LED_PIN, HIGH);
  }


  for (auto i = 0u; i < num_steppers; i++)
    {
      steppers[i]->runSpeed();
    }

}

void calibrate_lower_arm_stepper() 
{
  lower_arm_stepper.setCurrentPosition(stepper_lower_limits[0]);
}

void calibrate_upper_arm_stepper() 
{
  upper_arm_stepper.setCurrentPosition(stepper_upper_limits[1]);
}

void handle_packet(MotorPacket &packet)
{
  if (packet.checksum != packet.calculate_checksum())
  {
    packet.flag = MotorPacket::NACK;
  }
  else if (packet.flag == MotorPacket::MOT)
  {
    for (auto i = 0u; i < num_steppers; i++)
    {
      int cmd_vel = packet.stepper_vel[i];
      int cmd_pos = max(packet.stepper_pos[i], stepper_lower_limits[i]);
      cmd_pos = min(cmd_pos, stepper_upper_limits[i]);
      steppers[i]->moveTo(cmd_pos);
      steppers[i]->setSpeed(cmd_vel);
    }
  }
  else if (packet.flag == MotorPacket::ENC)
  {
    for (auto i = 0u; i < num_steppers; i++)
    {
      int pos = steppers[i]->currentPosition();
      packet.stepper_pos[i] = pos;
    }
  }
  else if (packet.flag == MotorPacket::HEY)
  {
    for (auto i = 0u; i < num_steppers; i++)
    {
      steppers[i]->setCurrentPosition(packet.stepper_pos[i]);
    }
  }
  packet.checksum = packet.calculate_checksum();
}