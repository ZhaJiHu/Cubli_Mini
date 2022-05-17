#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Config {  

#define USER_WIFI 1
#define USER_SERIAL 1
#define FIRST_DOWNLOAD_CODE 0
#define LOW_POWER_VOLTAGE_THRESHOLD 9.5
#define IMU_INIT_GYRO_THRESHOLD 4
#define IMU_STATIC_GYRO_THRESHOLD 5

#define WIFI_SSID "ChinaNet-RFPPQ9"
#define WIFI_PASSWORD "12345678"

#define WIFI_TCP_SERVER_IP "192.168.2.14"
#define WIFI_TCP_SERVER_PORT 1312

#define MOTOR_PP 7

#define MOTOR_1_PWM1_PIN (int)25
#define MOTOR_1_PWM2_PIN (int)26
#define MOTOR_1_PWM3_PIN (int)27
#define MOTOR_1_EN_PIN (int)33

#define MOTOR_1_SENSOR_I2C_SDA_PIN (int)4
#define MOTOR_1_SENSOR_I2C_SCL_PIN (int)14

#define CAN_TXD_PIN (int)15
#define CAN_RXD_PIN (int)13

#define IMU_SCL_PIN (int)18
#define IMU_SDA_PIN (int)5
#define IMU_INTERRUPT_PIN (int)23

#define BULE_LED_PIN (int)10
#define GREEN_LED_PIN (int)9
#define MOTOR_GREEN_LED_PIN (int)21

#define KEY_1_PIN (int)22
#define KEY_2_PIN (int)19

#define BAT_VOLTAGE_PIN (int)36

} // namespace Cubli 
} // namespace Config 