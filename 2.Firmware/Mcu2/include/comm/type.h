#pragma once
#include <Arduino.h>

namespace Cubli {
namespace Comm {

enum CanStatus_e
{
    eON_LINE = 0x01,
    eOFF_LINE = 0x00,
};

enum Result_e
{
    eSUCCESS = 0x01,
    eFail = 0x00
};

struct SendSpeed_t
{
    float ch2_speed;
    float ch3_speed;
};

struct SetSpeed_t
{
    float ch2_speed;
    float ch3_speed;
};

struct MotorControl_t
{
    SendSpeed_t send_speed;
    SetSpeed_t set_speed;
};

struct DeviceStatus_t
{
    Result_e ch2_motor_init_result;
    Result_e ch3_motor_init_result;
    CanStatus_e can_status;
};

} // namespace Cubli
} // namespace Comm