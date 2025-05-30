#ifndef RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP
#define RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP

#include <Arduino.h>

struct __attribute__((packed)) MotorPacket
{
  uint8_t header = 0x64;
  uint8_t id = 0x00;
  uint8_t flag;
  int8_t bldc[1];
  int8_t servo[1];
  int16_t stepper[2]; 
  uint8_t checksum;

  enum MotorType { BLDC, SERVO, STEPPER };

  enum Flag { ACK = 0xA1, NACK = 0xA2, ENC = 0xB1, MOT = 0xB2, HEY = 0xC1, CAL = 0xD1 };

  uint8_t calculate_checksum() const
  {
    uint8_t sum = header + id + flag;
    for (size_t i = 0; i < sizeof(bldc) / sizeof(bldc[0]); ++i)
    {
      sum += static_cast<uint8_t>(bldc[i]);
    }
    for (size_t i = 0; i < sizeof(servo) / sizeof(servo[0]); ++i) 
    {
      sum += static_cast<uint8_t>(servo[i] & 0xFF);     
      sum += static_cast<uint8_t>((servo[i] >> 8) & 0xFF);
    }
    for (size_t i = 0; i < sizeof(stepper) / sizeof(stepper[0]); ++i) 
    {
      sum += static_cast<uint8_t>(stepper[i] & 0xFF);      
      sum += static_cast<uint8_t>((stepper[i] >> 8) & 0xFF);
    }
    return sum;
  }
};

#endif // RUSHBOTT_HARDWARE_MOTOR_PACKET_HPP