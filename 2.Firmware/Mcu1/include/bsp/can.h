#pragma once
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include "config/config.h"

namespace CubliMini {
namespace Bsp {  

#define CAN_OFFLINE_COUNT 500 // 500次,0.5s

enum CanFrameId
{
    eCAN_SET_MOTOR_SPEED_FRAME = 0x100,
    eCAN_GET_MOTOR_SPEED_FRAME = 0x101,
    eCAN_GET_MOTOR_HEALT_FRAME = 0x102
};

enum CanStatus_e
{
    eON_LINE = 0x01,
    eOFF_LINE = 0x00,
};

class CanDriver
{
   public:
    CanDriver()
    {
        can_is_online_ = eOFF_LINE;
        rx_frame_count_ = 0;
    }
    void Init(int _can_txd_pin, int _can_rxd_pin, CAN_speed_t _can_speed);
    void CanSendMotorSpeed(float _set_ch2_speed, float _set_ch3_speed);
    void CanGetMsg(float & _get_ch2_speed, float & _get_ch3_speed);
    CanStatus_e CanIsOnline();
    CanStatus_e can_is_online_;
   private:
    uint32_t rx_frame_count_;
};

} // namespace Cubli 
} // namespace Bsp 