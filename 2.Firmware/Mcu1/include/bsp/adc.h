#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Bsp {  

class AdcDriver
{
    public:
        void Init(int _adc_pin)
        {
            adc_pin_ = _adc_pin;
            pinMode(adc_pin_, INPUT);
        }

        int GetAdcValue()
        {
            return analogRead(adc_pin_);
        }
    private:
        int adc_pin_;
};

} // namespace Cubli 
} // namespace Bsp 