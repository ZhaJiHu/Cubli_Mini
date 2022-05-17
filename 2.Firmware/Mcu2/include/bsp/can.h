#pragma once
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include "config/config.h"
#include "comm/type.h"

using namespace Cubli::Comm;

namespace Cubli {
namespace Bsp {

#define CAN_OFFLINE_COUNT 500 // 250次,0.5s

enum CanFrameId_e
{
    eCAN_SEND_MOTOR_SPEED_FRAME = 0x101,
    eCAN_GET_MOTOR_SPEED_FRAME = 0x100,
    eCAN_GET_MOTOR_HEALT_FRAME = 0x102
};

class CanDriver
{
   public:
    CanDriver()
    {
        rx_frame_count_ = 0;
        can_is_online_ = eOFF_LINE;
    }
    void Init(int _can_txd_pin, int _can_rxd_pin, CAN_speed_t _can_speed);
    void CanSendMotorSpeed(float set_ch1_speed, float set_ch2_speed);
    void CanGetMotorSpeed(float &_get_ch1_speed, float &_get_ch2_speed);
    CanStatus_e CanIsOnline();
    CanStatus_e can_is_online_;
   private:
    uint32_t rx_frame_count_;
};

} // namespace Cubli
} // namespace Bsp