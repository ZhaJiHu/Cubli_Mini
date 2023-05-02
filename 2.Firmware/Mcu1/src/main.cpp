#include <Arduino.h>
#include <SimpleFOC.h>
#include <esp_wifi.h>

#include "bsp/bsp_driver.h"
#include "bsp/can.h"
#include "control/cubli_mini.h"
#include "control/param.h"
#include "control/serial_commander.h"
#include "control/wifi_commander.h"
#include "imu/ahrs.h"

using namespace CubliMini::Imu;
using namespace CubliMini::ImuDriver;
using namespace CubliMini::Control;

AHRS ahrs;
SerialCommander serial_commander;
CubliMiniControl cubli_mini;
WifiCommander wifi_commander;

MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(MOTOR_PP);
// motor1
BLDCDriver3PWM driver1 =
    BLDCDriver3PWM(MOTOR_1_PWM1_PIN, MOTOR_1_PWM2_PIN, MOTOR_1_PWM3_PIN, MOTOR_1_EN_PIN);
// i2c
TwoWire motor1_i2c = TwoWire(0);
TwoWire imu_i2c    = TwoWire(1);

SemaphoreHandle_t x_mutex;

void CanTask(void *parameter);
void WifiCommanderTask(void *parameter);
void SensorUpdateTask(void *parameter);
void ModeTask(void *parameter);
void SimpleFocInit();

void setup()
{
    Serial.begin(115200);
    delay(3000);
    ahrs.mpu6050_driver.Init(&imu_i2c);
    cubli_mini.LinkSensor(&ahrs);
    cubli_mini.Init();
    SimpleFocInit();

    // core 1
    xTaskCreatePinnedToCore(SensorUpdateTask, "ImulTask", 5000, NULL, 5, NULL, 1);

    if (cubli_mini.wifi_param_.use_wifi)
    {
        wifi_commander.SetOption(cubli_mini.wifi_param_);
        wifi_commander.ConnectWifi();
        if (wifi_commander.GetConnectStatus() == WifiConnectStatus_e::eWIFI_SUCCESS)
        {
            // core 0
            xTaskCreatePinnedToCore(
                WifiCommanderTask, "WifiCommanderTask", 30000, NULL, 7, NULL, 0);
        }
    }

    // core 1
    xTaskCreatePinnedToCore(ModeTask, "ModeTask", 5000, NULL, 2, NULL, 1);

    printf("Cubli_Mini ready. \r\n");
    delay(1000);
}

void SimpleFocInit()
{
    motor1_i2c.begin(MOTOR_1_SENSOR_I2C_SDA_PIN, MOTOR_1_SENSOR_I2C_SCL_PIN, (uint32_t)400000);
    sensor1.init(&motor1_i2c);
    motor1.linkSensor(&sensor1);
    driver1.voltage_power_supply = 12;
    driver1.init();
    motor1.linkDriver(&driver1);
    motor1.controller    = MotionControlType::torque;
    motor1.voltage_limit = 12;
    motor1.useMonitoring(Serial);
    motor1.monitor_downsample = 0;
    motor1.init();
    uint8_t res = motor1.initFOC();
    cubli_mini.SetCh1MotorInitResult(res);
}

// core 1
void loop()
{
    motor1.loopFOC();
    motor1.move(cubli_mini.motor_set_speed_.ch1);
    cubli_mini.motor_get_speed_.ch1 = -motor1.shaft_velocity;
}

float angle_z = 0;

void ModeTask(void *parameter)
{
    Time time;
    for (;;)
    {
        cubli_mini.ModeControl();
        serial_commander.Run(Serial, cubli_mini);
        cubli_mini.PBalanceAutoCalibration();
        cubli_mini.UBalanceAutoCalibration();
#if 0
        printf("%0.2f %0.2f %0.2f\r\n", 
            cubli_mini.p_balance_.sensor.x.angle,
            cubli_mini.p_balance_.sensor.y.angle,
            cubli_mini.p_balance_.sensor.z.angle);
#endif
        delay(50);
    }
}

void WifiCommanderTask(void *parameter)
{
    for (;;)
    {
        wifi_commander.WaitClientConnect();
        wifi_commander.TcpClientUnpack(cubli_mini);
    }
}

void SensorUpdateTask(void *parameter)
{
    uint64_t count         = 0;
    static float last_gyro = 0;
    for (;;)
    {
        count++;
        cubli_mini.SensorUpdate();

        if (count % 2 == 0)
        {
            cubli_mini.Control();
        }
        delay(1);
    }
}