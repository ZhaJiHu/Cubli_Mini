#include <Arduino.h>
#include <SimpleFOC.h>
#include "imu/ahrs.h"
#include "bsp/bsp_driver.h"
#include "bsp/can.h"
#include "control/serial_commander.h"
#include "control/cubli_mini.h"
#include "control/wifi_commander.h"
#include "control/param.h"

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
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTOR_1_PWM1_PIN, MOTOR_1_PWM2_PIN, MOTOR_1_PWM3_PIN, MOTOR_1_EN_PIN);
// i2c
TwoWire motor1_i2c = TwoWire(0);
TwoWire imu_i2c = TwoWire(1);

SemaphoreHandle_t x_mutex;

float ch1_target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor1, cmd); }
void ch1doTarget(char* cmd) { command.scalar(&ch1_target_velocity, cmd); }

void CanTask(void *parameter);
void WifiCommanderTask(void *parameter);
void ControlTask(void *parameter);
void ModecontrolTask(void *parameter);
void SerialCmdTask(void *parameter);
void SensorUpdateTask(void *parameter);
void SerialLogTask(void *parameter);
void SimpleFocInit();


void setup()
{
  Serial.begin(115200);
  ahrs.mpu6050_driver.Init(&imu_i2c);
  cubli_mini.LinkSensor(&ahrs);
  cubli_mini.Init();
  SimpleFocInit();
  x_mutex = xSemaphoreCreateMutex();
#if USER_WIFI
  wifi_commander.ConnectWifi();
  if(wifi_commander.GetConnectStatus() == WifiConnectStatus_e::eWIFI_SUCCESS)
  {
      wifi_commander.LinkMutex(&x_mutex);
  }
#endif
#if FIRST_DOWNLOAD_CODE
  // 第一次烧录芯片的时候需要
  cubli_mini.FirstWriteParamToEeprom();
#endif 
  // core 0
#if USER_WIFI
  if(wifi_commander.GetConnectStatus() == WifiConnectStatus_e::eWIFI_SUCCESS)
  {
      xTaskCreatePinnedToCore(WifiCommanderTask, "WifiCommanderTask", 20000, NULL, 6, NULL, 0);
  }
#endif

  // core 1
  xTaskCreatePinnedToCore(ModecontrolTask, "ModecontrolTask", 10000, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(SerialLogTask, "SerialLogTask", 10000, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(SerialCmdTask, "SerialCmdTask", 10000, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 20000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(SensorUpdateTask, "ImulTask", 10000, NULL, 2, NULL, 1);

  command.add('T', ch1doTarget, "ch1 target voltage");
  printf("Cubli_Mini ready. \r\n");
  _delay(1000);
}

void SimpleFocInit()
{
    motor1_i2c.begin(MOTOR_1_SENSOR_I2C_SDA_PIN, MOTOR_1_SENSOR_I2C_SCL_PIN, (uint32_t)400000);
    sensor1.init(&motor1_i2c);
    motor1.linkSensor(&sensor1);
    driver1.voltage_power_supply = 12;
    driver1.init();
    motor1.linkDriver(&driver1);
    motor1.controller = MotionControlType::torque;
    motor1.voltage_limit = 12;
    motor1.useMonitoring(Serial);
    motor1.monitor_downsample = 0;
    motor1.init();
    uint8_t res = motor1.initFOC();
    cubli_mini.SetCh1MotorInitResult(res);
    command.add('M',doMotor,"motor");
}

// core 1
void loop()
{
  motor1.loopFOC();
  motor1.move(cubli_mini.motor_set_speed_.ch1);
  cubli_mini.motor_get_speed_.ch1 = motor1.shaft_velocity;
}

void SerialCmdTask(void *parameter)
{
  for(;;)
  {
#if 0
    command.run();
    motor1.monitor();
#endif
#if USER_SERIAL
    xSemaphoreTake(x_mutex, portMAX_DELAY);
    serial_commander.Run(Serial, cubli_mini.p_balance_.param, cubli_mini.u_ch2_balance_.param);
    if(serial_commander.output_angle_)
    {
        printf("x_A:%f y_A:%f z_A:%f\r\n",
                cubli_mini.p_balance_.sensor.x.angle, 
                cubli_mini.p_balance_.sensor.y.angle,
                cubli_mini.p_balance_.sensor.z.angle);
    }
#if USER_WIFI
    if(wifi_commander.GetConnectStatus() == WifiConnectStatus_e::eWIFI_SUCCESS)
    {
      wifi_commander.sensor_angle_.x = cubli_mini.p_balance_.sensor.x.angle;
      wifi_commander.sensor_angle_.y = cubli_mini.p_balance_.sensor.y.angle;
      wifi_commander.sensor_angle_.z = cubli_mini.p_balance_.sensor.z.angle;
    }
#endif
    xSemaphoreGive(x_mutex);	
#endif 
    _delay(20);
  }
}

void SerialLogTask(void *parameter)
{
    for(;;)
    {
        #if 0
            sensor1.update();
            _delay(1);
            printf("ch1_Angle: %f\r\n", sensor1.getAngle());
        #endif
#if 0
        printf("get speed ch1:%f ch2:%f ch3:%f\r\n", 
        cubli_mini.motor_set_speed_.ch1, 
        cubli_mini.motor_set_speed_.ch2, 
        cubli_mini.motor_set_speed_.ch3);
#endif
#if 0
        printf("angle x:%f y:%f z:%f\r\n", 
        cubli_mini.p_balance_.sensor.x, 
        cubli_mini.p_balance_.sensor.y, 
        cubli_mini.p_balance_.sensor.z);
#endif
        delay(100);
    }
}

#if USER_WIFI
void WifiCommanderTask(void *parameter)
{
  for(;;)
  {
    wifi_commander.TcpTask(cubli_mini.p_balance_.param, cubli_mini.u_ch2_balance_.param);
    delay(2000);
  }
}
#endif

void ModecontrolTask(void *parameter)
{
  for(;;)
  {
    cubli_mini.ModeControl();
    _delay(20);
  }
}

void SensorUpdateTask(void *parameter)
{
  for(;;)
  {
    cubli_mini.SensorUpdate();
    _delay(1);
  }
}

void ControlTask(void *parameter)
{
  for(;;)
  {
    cubli_mini.Control();
    _delay(2);
  }
}