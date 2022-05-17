#include "bsp/led.h"

namespace Cubli
{
namespace Bsp
{

void LedDriver::Init(int _led_pin)
{
    led_pin_ = _led_pin;
    pinMode(led_pin_, OUTPUT);
    digitalWrite(led_pin_, LED_OFF);
}

void LedDriver::AlwaysOn()
{
    digitalWrite(led_pin_, LED_ON);
}

void LedDriver::AlwaysOff()
{
    digitalWrite(led_pin_, LED_OFF);
}

/*
    *  输入1：_flashes_interval_time， 闪烁的间隔时间 , ms
    *  输入2：_number_of_flashes，闪烁的次数, ms
    *  输入3：_control_cycle，一个控制周期, ms
    */
void LedDriver::Control(int _flashes_interval_time, int _number_of_flashes, int _control_cycle)
{
    float time_interval = time_.GetTimeMs() * 1000.0f; // ms
    control_cycle_time_count_ += time_interval;
    flashes_interva_time_count_ += time_interval;

    if (control_cycle_time_count_ <= _control_cycle)
    {
        if (flashes_interva_time_count_ >= _flashes_interval_time)
        {
            if (number_of_flashes_count_ < _number_of_flashes * 2)
            {
                flashes_interva_time_count_ = 0;
                number_of_flashes_count_++;
                inversion_status_ = !inversion_status_;
                if (inversion_status_)
                {
                    digitalWrite(led_pin_, LED_ON);
                }
                else
                {
                    digitalWrite(led_pin_, LED_OFF);
                }
            }
            else
            {
                digitalWrite(led_pin_, LED_OFF);
                inversion_status_ = false;
            }
        }
    }
    else
    {
        control_cycle_time_count_ = 0;
        flashes_interva_time_count_ = 0;
        number_of_flashes_count_ = 0;
        digitalWrite(led_pin_, LED_OFF);
        inversion_status_ = false;
    }
}

} // namespace Cubli
} // namespace Bsp