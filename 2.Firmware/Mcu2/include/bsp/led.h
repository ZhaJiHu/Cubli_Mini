#pragma once
#include <Arduino.h>
#include "comm/time.h"

namespace Cubli {
namespace Bsp {

#define LED_ON LOW
#define LED_OFF HIGH

class LedDriver
{
   public:
    LedDriver()
    {
        control_cycle_time_count_ = 0.0f;
        flashes_interva_time_count_ = 0.0f;
        number_of_flashes_count_ = 0;
        inversion_status_ = false;
    }
    void Init(int _led_pin);
    void Control(int _flashes_interval_time, int _number_of_flashes, int _control_cycle);
    void AlwaysOn();
    void AlwaysOff();

   private:
    int led_pin_;
    Comm::Time time_;
    // 累计一个周期内的时间
    float control_cycle_time_count_;
    // 一个闪烁周期内累积的时间
    float flashes_interva_time_count_;
    // 累计闪烁的次数，一个闪烁等于两次电平翻转
    int number_of_flashes_count_;
    // 电平状态
    bool inversion_status_;
};

} // namespace Cubli
} // namespace Bsp