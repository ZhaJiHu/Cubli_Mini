#pragma once
#include <Arduino.h>
#include "imu/ahrs.h"
#include "bsp/bsp_driver.h"
#include "control/param.h"

using namespace CubliMini::Imu;
using namespace CubliMini::Comm;
using namespace CubliMini::Bsp;

namespace CubliMini {
namespace Control {

#define STATIC_ANGLE_PROTECTION_THRESHOLD 5
#define BALABCE_PROTECTION_THRESHOLD 15
#define VOLTAGE_LIMIT 11

enum MotorIintResult_e
{
  eSUCCESS = 0x01,
  eFail = 0x00
};

enum DeviceHealt_e
{
  eDEVICE_ONLINE = 0x03,
  eDEVICE_MOTOR1_OFFLINE = 0x02,
  eDEVICE_CAN_OFFLINE = 0x01,
  eDEVICE_ALL_OFFLINE = 0x00
};


struct MotorInitStatus_t
{
  uint8_t ch1;
  uint8_t ch2;
  uint8_t ch3;
};

struct MotorGetSpeed_t
{
  float ch1;
  float ch2;
  float ch3;
};

struct MotorSetSpeed_t
{
  float ch1;
  float ch2;
  float ch3;
};

enum ControlMode_e
{
    eNORMAL = 0x00,
    ePOINT_BALANCE = 0x01,
    eUNILATERAL_BALANCE = 0x02,
    ePOINT_CALIBRATION = 0x03,
    eUNILATERA_CALIBRATION = 0x04,
    eMOTOR_TEST = 0x05
} ;

class CubliMiniControl
{
   public:
    void Init();
    void LinkSensor(AHRS *_ahrs);
    void SensorUpdate();
    void ModeControl();
    void Control();
    void FirstWriteParamToEeprom();
  
   public:
    BspDriver bsp_driver_;
    MotorSetSpeed_t motor_set_speed_;
    MotorGetSpeed_t motor_get_speed_;
    UBalance_t u_ch2_balance_;
    PBalance_t p_balance_;
    void SetCh1MotorInitResult(uint8_t _result);
  
   private:
    float LqrCalAxis(const AxisSensor_t &_sensor, const AxisParam_t &_param, float _axis_offset);
    float Lqr(const AxisSensor_t &_sensor, const AxisParam_t &_param);
    void Ch2AxisUBalance();
    void Ch2AxisUBalanceParamInit();
    void PBalanceParamInit();
    void PBalance();
    void MotorStop();
    float Limit(float _data, float _limit);
    void GetAxisMoveSpeed( const MotorGetSpeed_t & _get_speed, 
                            float &_x_speed, 
                            float &_y_speed, 
                            float &_z_speed);
    // void GetMotorMoveSpeed( float _x_control, 
    //                         float _y_control, 
    //                         float _z_control, 
    //                         MotorSetSpeed_t & _set_speed);

    void GetMotorMoveSpeed(   float _x_speed, 
                              float _y_speed, 
                              float _z_speed, 
                              MotorSetSpeed_t & _set_speed);
  
   private:
    float protection_threshold_;
    bool cube_is_upside_dowm_;
    bool motor_test_start_;
    ControlMode_e control_mode_;
    AHRS *ahrs_;
    bool systeam_low_power_;
    Time low_power_reminder_time_;
    DeviceHealt_e mcu1_device_healt_;
    MotorInitStatus_t motor_init_status_;
};
  

} // namespace Cubli 
} // namespace Control 