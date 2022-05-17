#include <Arduino.h>
#include <SimpleFOC.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include "config/config.h"
#include "bsp/bsp_driver.h"
#include "comm/type.h"

using namespace Cubli::Comm;
using namespace Cubli::Bsp;
using namespace Cubli::Config;

MotorControl_t motor_control;
DeviceStatus_t device_status;

BspDriver bsp_driver;
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire i2c2 = TwoWire(0);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);
TwoWire i2c3 = TwoWire(1);

BLDCMotor motor2 = BLDCMotor(MOTOR_2_PP);
BLDCMotor motor3 = BLDCMotor(MOTOR_3_PP);

BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTOR_2_PWM1_PIN, MOTOR_2_PWM2_PIN, MOTOR_2_PWM3_PIN, MOTOR_2_EN_PIN);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(MOTOR_3_PWM1_PIN, MOTOR_3_PWM2_PIN, MOTOR_3_PWM3_PIN, MOTOR_3_EN_PIN);

void CanTask(void *parameter);
void LedTask(void *parameter);
void SimpleFocInit();

void setup()
{
  Serial.begin(115200);

  memset(&motor_control, 0, sizeof(motor_control));
  memset(&device_status, 0, sizeof(device_status));

  bsp_driver.Init();
  SimpleFocInit();

  xTaskCreatePinnedToCore(CanTask, "can tx task", 10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(LedTask, "led task", 10000, NULL, 3, NULL, 1);

  printf("Motor ready.");
  _delay(1000);
}

void SimpleFocInit()
{
  i2c2.begin(MOTOR_2_SENSOR_I2C_SDA_PIN, MOTOR_2_SENSOR_I2C_SCL_PIN, (uint32_t)400000);
  sensor2.init(&i2c2);

  i2c3.begin(MOTOR_3_SENSOR_I2C_SDA_PIN, MOTOR_3_SENSOR_I2C_SCL_PIN, (uint32_t)400000);
  sensor3.init(&i2c3);

  motor2.linkSensor(&sensor2);
  motor3.linkSensor(&sensor3);

  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  driver3.voltage_power_supply = 12;
  driver3.init();
  motor3.linkDriver(&driver3);

  motor2.controller = MotionControlType::torque;
  motor3.controller = MotionControlType::torque;
  motor2.voltage_limit = 12;
  motor3.voltage_limit = 12;

  motor2.useMonitoring(Serial);
  motor3.useMonitoring(Serial);

  motor2.init();
  if (motor2.initFOC() == Result_e::eSUCCESS)
  {
    device_status.ch2_motor_init_result = Result_e::eSUCCESS;
  }
  else
  {
    device_status.ch2_motor_init_result = Result_e::eFail;
  }

  motor3.init();
  if (motor3.initFOC() == Result_e::eSUCCESS)
  {
    device_status.ch3_motor_init_result = Result_e::eSUCCESS;
  }
  else
  {
    device_status.ch3_motor_init_result = Result_e::eFail;
  }
}

void CanTask(void *parameter)
{
  for (;;)
  {
    bsp_driver.can_.CanSendMotorSpeed(motor_control.send_speed.ch2_speed, motor_control.send_speed.ch3_speed);
    bsp_driver.can_.CanGetMotorSpeed(motor_control.set_speed.ch2_speed, motor_control.set_speed.ch3_speed);
    device_status.can_status = bsp_driver.can_.CanIsOnline();
    if (device_status.can_status == CanStatus_e::eOFF_LINE)
    {
        motor_control.set_speed.ch2_speed = motor_control.set_speed.ch3_speed = 0;
    }
    _delay(2);
  }
}

// LED指示灯优先级, 电机状态 > CAN掉线 > 正常
// bit 2: bit 1: bit 0
//   can   ch3    ch2
void LedTask(void *parameter)
{
  for (;;)
  {
    uint8_t status = (uint8_t)device_status.can_status << 2 | (uint8_t)device_status.ch3_motor_init_result << 1 | (uint8_t)device_status.ch2_motor_init_result;
    switch (status)
    {
    // 所有设备正常
    case 0x07:
      bsp_driver.motor_green_led_.Control(200, 1, 3000);
      break;
    // 设备有问题
    default:
      bsp_driver.motor_green_led_.Control(200, 3, 3000);
    }
    _delay(100);
  }
}

void loop()
{
  motor2.loopFOC();
  motor3.loopFOC();
  motor2.move(motor_control.set_speed.ch2_speed);
  motor3.move(motor_control.set_speed.ch3_speed);

  motor_control.send_speed.ch2_speed = motor2.shaft_velocity;
  motor_control.send_speed.ch3_speed = motor3.shaft_velocity;
// read sensor value
#if 0
    sensor2.update();
    sensor3.update();
     _delay(1);
    printf("ch1_Angle: %f, ch2_Angle: %f \r\n", sensor2.getAngle(), sensor3.getAngle());
    printf(" speed1: %d, speed2: %d \r\n", sensor2.getVelocity(), sensor3.getVelocity());
#endif
}
