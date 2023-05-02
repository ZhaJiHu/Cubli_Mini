#pragma once
#include <Arduino.h>

#include "bsp/bsp_driver.h"
#include "comm/time.h"
#include "control/param.h"
#include "imu/ahrs.h"

using namespace CubliMini::Imu;
using namespace CubliMini::Comm;
using namespace CubliMini::Bsp;

namespace CubliMini {
namespace Control {

#define STATIC_ANGLE_PROTECTION_THRESHOLD 5
#define BALABCE_PROTECTION_THRESHOLD      15
// 电池输出有限，限制电机电压，降低功率
#define VOLTAGE_LIMIT 5

// 转速为5，经验值，旋转比较缓慢
#define AUTO_CALIBRATION_SPEED_THRESHOLD 5

#define AUTO_CALIBRATION_STATIC_THRESHOLD 5
#define AUTO_CALIBRATION_ANGLE_SCALE      0.1

#define AUTO_P_CALIBRATION_X_TIMEOUT   (1000 * 10)  // 10s
#define AUTO_P_CALIBRATION_Y_TIMEOUT   (1000 * 10)  // 10s
#define AUTO_P_CALIBRATION_PERIOD      400          // 350ms
#define AUTO_P_CALIBRATION_WAIT_STATIC 5000         // 5s

#define AUTO_U_CALIBRATION_PERIOD      400   // 350ms
#define AUTO_U_CALIBRATION_WAIT_STATIC 5000  // 5s

enum MotorIintResult_e
{
    eSUCCESS = 0x01,
    eFail    = 0x00
};

enum DeviceHealt_e
{
    eDEVICE_ONLINE         = 0x03,
    eDEVICE_MOTOR1_OFFLINE = 0x02,
    eDEVICE_CAN_OFFLINE    = 0x01,
    eDEVICE_ALL_OFFLINE    = 0x00
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
    eNORMAL                   = 0x00,
    ePOINT_BALANCE            = 0x01,
    ePOINT_CALIBRATION        = 0x03,
    ePOINT_CALIBRATIONING     = 0x06,
    eUNILATERA_BALANCE        = 0x02,
    eUNILATERA_CALIBRATION    = 0x04,
    eUNILATERA_CALIBRATIONING = 0x07,
    eMOTOR_TEST               = 0x05
};

enum CalibrationMode_e
{
    eX_AXIS = 0x01,
    eY_AXIS = 0x02,
    eEND    = 0x03
};

class CubliMiniControl
{
   public:
    void Init();
    void LinkSensor(AHRS *_ahrs);
    void SensorUpdate();
    void ModeControl();
    void Control();
    void FirstWriteParamToEeprom();
    void ResetCanQueue();
    void FirstDownloadFlash();
    void WifiWriteDefaultParam();

   public:
    float z_speed_;

    BspDriver bsp_driver_;
    MotorSetSpeed_t motor_set_speed_;
    MotorGetSpeed_t motor_get_speed_;
    UBalance_t u_ch2_balance_;
    PBalance_t p_balance_;
    WifiParam_t wifi_param_;
    DynamicParam_t dynamic_param_;

    void SetCh1MotorInitResult(uint8_t _result);

    void PBalanceAutoCalibration();
    void StartPBalanceAutoCalibration();

    void StartUBalanceAutoCalibration();
    void UBalanceAutoCalibration();

   private:
    float LqrCalAxis(const AxisSensor_t &_sensor, const AxisParam_t &_param, float _axis_offset);
    float Lqr(const AxisSensor_t &_sensor, const AxisParam_t &_param);
    void Ch2AxisUBalance();
    void Ch2AxisUBalanceWriteDefaultParam();
    void PBalanceWriteDefaultParam();
    void PBalance();
    void MotorStop();
    float Limit(float _data, float _limit);
    void GetAxisMoveSpeed(
        const MotorGetSpeed_t &_get_speed, float &_x_speed, float &_y_speed, float &_z_speed);

    void GetMotorMoveSpeed(
        float _x_speed, float _y_speed, float _z_speed, MotorSetSpeed_t &_set_speed);

    void PBalanceCalibrationAngle_X(CalibrationMode_e &mode);
    void PBalanceCalibrationAngle_Y(CalibrationMode_e &mode);
    void UBalanceCalibrationAngle_X(CalibrationMode_e &mode);

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

    // p
    Time pbalance_timeout_y_;
    Time pbalance_timeout_x_;
    Time pbalance_calibration_period_;

    CalibrationMode_e pbalance_calibration_mode_;
    bool start_pbalance_calibration_;

    Time wait_p_calibration_;

    // u
    Time ubalance_timeout_x_;
    Time ubalance_calibration_period_;

    CalibrationMode_e ubalance_calibration_mode_;
    bool start_ubalance_calibration_;
    Time wait_u_calibration_;
};

}  // namespace Control
}  // namespace CubliMini