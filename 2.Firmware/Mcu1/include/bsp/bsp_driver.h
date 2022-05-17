#pragma once
#include <Arduino.h>
#include "bsp/led.h"
#include "bsp/key.h"
#include "bsp/adc.h"
#include "bsp/can.h"
#include "config/config.h"

using namespace CubliMini::Config;
namespace CubliMini {
namespace Bsp {  

class BspDriver
{
    public:
        void Init()
        {
            bule_led_.Init(BULE_LED_PIN);
            green_led_.Init(GREEN_LED_PIN);
            motor_green_led_.Init(MOTOR_GREEN_LED_PIN);

            key1_.Init(KEY_1_PIN);
            key2_.Init(KEY_2_PIN);

            bat_voltage_.Init(BAT_VOLTAGE_PIN);

            can_.Init(CAN_TXD_PIN, CAN_RXD_PIN, CAN_SPEED_1000KBPS);
        }
        LedDriver bule_led_;
        LedDriver green_led_;
        LedDriver motor_green_led_;

        KeyDriver key1_;
        KeyDriver key2_;

        AdcDriver bat_voltage_;

        CanDriver can_;

        float GetBatVoltage()
        {
            int adc_data = bat_voltage_.GetAdcValue();
            return (float)adc_data  / (4096.0f / 3.3f) / (10.0f / (42.2f + 10.0f));
        }
};

} // namespace Cubli 
} // namespace Bsp 