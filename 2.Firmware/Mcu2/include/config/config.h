#pragma once
#include <Arduino.h>

namespace Cubli {
namespace Config {

#define MOTOR_2_PP 7

#define MOTOR_2_PWM1_PIN (int)25
#define MOTOR_2_PWM2_PIN (int)26
#define MOTOR_2_PWM3_PIN (int)27
#define MOTOR_2_EN_PIN (int)33

#define MOTOR_2_SENSOR_I2C_SDA_PIN (int)4
#define MOTOR_2_SENSOR_I2C_SCL_PIN (int)14

#define MOTOR_3_PP 7

#define MOTOR_3_PWM1_PIN (int)22
#define MOTOR_3_PWM2_PIN (int)19
#define MOTOR_3_PWM3_PIN (int)23
#define MOTOR_3_EN_PIN (int)21

#define MOTOR_3_SENSOR_I2C_SDA_PIN (int)5
#define MOTOR_3_SENSOR_I2C_SCL_PIN (int)18

#define CAN_TXD_PIN (int)15
#define CAN_RXD_PIN (int)13

#define MOTOR_GREEN_LED_PIN (int)10

} // namespace Cubli
} // namespace Config