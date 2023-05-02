#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Config {

#define USER_SERIAL                 1
#define LOW_POWER_VOLTAGE_THRESHOLD 8
#define IMU_INIT_GYRO_THRESHOLD     10
#define IMU_STATIC_GYRO_THRESHOLD   8

#define WIFI_DEFAULT_SSID            "TeLunSu"
#define WIFI_DEFAULT_PASSWORD        "12345678"
#define WIFI_DEFAULT_TCP_SERVER_PORT 1312

#define P_BALANCE_X_P -8
#define P_BALANCE_X_V 0.5f
#define P_BALANCE_X_S 0.12f
#define P_BALANCE_X_A 0.5f

#define P_BALANCE_Y_P 8
#define P_BALANCE_Y_V 0.6f
#define P_BALANCE_Y_S 0.12f
#define P_BALANCE_Y_A 1.5f

#define P_BALANCE_Z_P 0
#define P_BALANCE_Z_V -0.3f
#define P_BALANCE_Z_S 0.025f
#define P_BALANCE_Z_A 0

#define U_BALANCE_CH2_P -10
#define U_BALANCE_CH2_V 0.55f
#define U_BALANCE_CH2_S 0.25f
#define U_BALANCE_CH2_A 1.21f

#define MOTOR_PP 7

#define MOTOR_1_PWM1_PIN (int)25
#define MOTOR_1_PWM2_PIN (int)26
#define MOTOR_1_PWM3_PIN (int)27
#define MOTOR_1_EN_PIN   (int)33

#define MOTOR_1_SENSOR_I2C_SDA_PIN (int)4
#define MOTOR_1_SENSOR_I2C_SCL_PIN (int)14

#define CAN_TXD_PIN (int)15
#define CAN_RXD_PIN (int)13

#define IMU_SCL_PIN       (int)18
#define IMU_SDA_PIN       (int)5
#define IMU_INTERRUPT_PIN (int)23

#define BULE_LED_PIN        (int)10
#define GREEN_LED_PIN       (int)9
#define MOTOR_GREEN_LED_PIN (int)21

#define KEY_1_PIN (int)22
#define KEY_2_PIN (int)19

#define BAT_VOLTAGE_PIN (int)36

}  // namespace Config
}  // namespace CubliMini