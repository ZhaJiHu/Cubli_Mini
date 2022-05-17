#include "control/cubli_mini.h"

namespace CubliMini {
namespace Control {

void CubliMiniControl::LinkSensor(AHRS *_ahrs)
{
    ahrs_ = _ahrs;
}

void CubliMiniControl::Init()
{
    ParamInit(p_balance_.param, u_ch2_balance_.param);
    control_mode_ = eNORMAL;
    cube_is_upside_dowm_ = true;
    motor_test_start_ = false;
    systeam_low_power_ = false;
    protection_threshold_ = BALABCE_PROTECTION_THRESHOLD;
    mcu1_device_healt_ = eDEVICE_ALL_OFFLINE;
    memset(&motor_set_speed_, 0, sizeof(motor_set_speed_));
    bsp_driver_.Init();
}

void CubliMiniControl::GetAxisMoveSpeed(const MotorGetSpeed_t & _get_speed, 
                                    float &_x_speed, 
                                    float &_y_speed, 
                                    float &_z_speed)
{
    _x_speed = _get_speed.ch2 - _get_speed.ch3 * sin((30) / 57.3f) - _get_speed.ch1 * sin((30) / 57.3f);
    _y_speed = (_get_speed.ch3 - _get_speed.ch1) * cos((30) / 57.3f);
    _z_speed = (_get_speed.ch2 + _get_speed.ch3 + _get_speed.ch1);
}

void CubliMiniControl::GetMotorMoveSpeed(   float _x_speed, 
                                        float _y_speed, 
                                        float _z_speed, 
                                        MotorSetSpeed_t & _set_speed)
{
    _set_speed.ch1 = - _y_speed * cos((30) / 57.3f) - _x_speed * sin((30) / 57.3f) + _z_speed / 3.0f ;
    _set_speed.ch2 = _x_speed + _z_speed  / 3.0f;
    _set_speed.ch3 =  -_x_speed * sin((30) / 57.3f) + _y_speed * cos((30) / 57.3f) + _z_speed / 3.0f;
}


float CubliMiniControl::Lqr(const AxisSensor_t &_sensor, const AxisParam_t &_param)
{
    return (_sensor.angle - _param.angle_offset) * _param.kp + _sensor.gyro * _param.kv + _sensor.speed * _param.ks;
}

float CubliMiniControl::LqrCalAxis(const AxisSensor_t &_sensor, const AxisParam_t &_param, float _axis_offset)
{
    return (_sensor.angle * cos(_axis_offset / 57.3f) - _param.angle_offset) * _param.kp + _sensor.gyro * cos(_axis_offset / 57.3f) * _param.kv + _sensor.speed * _param.ks;
}

void CubliMiniControl::Ch2AxisUBalance()
{
    if(ahrs_->is_static_)
    {
        cube_is_upside_dowm_ = false;
    }

    if(fabs(u_ch2_balance_.sensor.angle - u_ch2_balance_.param.angle_offset) < protection_threshold_ &&
       cube_is_upside_dowm_ == false)
    {
        u_ch2_balance_.sensor.speed = motor_get_speed_.ch2;
        motor_set_speed_.ch2 = Lqr(u_ch2_balance_.sensor, u_ch2_balance_.param);
    }
    else
    {
        cube_is_upside_dowm_ = true;
        motor_set_speed_.ch2 = 0;
    }
    motor_set_speed_.ch1 = 0;
    motor_set_speed_.ch3 = 0;
    motor_set_speed_.ch2 = Limit(motor_set_speed_.ch2, VOLTAGE_LIMIT);
}

void CubliMiniControl::PBalance()
{
    if(ahrs_->is_static_)
    {
        cube_is_upside_dowm_ = false;
        p_balance_.param.z.angle_offset = p_balance_.sensor.z.angle;
    }

    if( fabs(p_balance_.sensor.x.angle - p_balance_.param.x.angle_offset) < protection_threshold_ && 
        fabs(p_balance_.sensor.y.angle - p_balance_.param.y.angle_offset) < protection_threshold_ &&
        cube_is_upside_dowm_ == false)
    {
        GetAxisMoveSpeed(   motor_get_speed_,
                            p_balance_.sensor.x.speed, 
                            p_balance_.sensor.y.speed, 
                            p_balance_.sensor.z.speed);

        float x_axis_speed = Lqr(p_balance_.sensor.x, p_balance_.param.x);
        float y_axis_speed = Lqr(p_balance_.sensor.y, p_balance_.param.y);
        float z_axis_speed = Lqr(p_balance_.sensor.z, p_balance_.param.z);

        GetMotorMoveSpeed(  x_axis_speed, 
                            y_axis_speed, 
                            z_axis_speed, 
                            motor_set_speed_);
    }
    else
    {
        cube_is_upside_dowm_ = true;
        motor_set_speed_.ch1 = 0;
        motor_set_speed_.ch2 = 0;
        motor_set_speed_.ch3 = 0;
    }

    motor_set_speed_.ch1 = Limit(motor_set_speed_.ch1, VOLTAGE_LIMIT);
    motor_set_speed_.ch2 = Limit(motor_set_speed_.ch2, VOLTAGE_LIMIT);
    motor_set_speed_.ch3 = Limit(motor_set_speed_.ch3, VOLTAGE_LIMIT);
}

void CubliMiniControl::Control()
{
    if( systeam_low_power_ == false)
    {
        switch (control_mode_)
        {
        case eNORMAL:
                MotorStop();
            break;
        case ePOINT_BALANCE:
                    PBalance();
            break;
        case eUNILATERAL_BALANCE:
                    Ch2AxisUBalance();
            break;
        case ePOINT_CALIBRATION:
                if(ahrs_->is_static_ && fabs(p_balance_.sensor.x.angle) < 4 &&  fabs(p_balance_.sensor.y.angle) < 4)
                {
                    p_balance_.param.x.angle_offset = p_balance_.sensor.x.angle;
                    p_balance_.param.y.angle_offset = p_balance_.sensor.y.angle;
                    p_balance_.param.z.angle_offset = p_balance_.sensor.z.angle;
                    printf("POINT ALIBRATION succeeded !!!");
                    control_mode_ = eNORMAL;
                } 
                MotorStop();
            break;
        case eUNILATERA_CALIBRATION:
                if(ahrs_->is_static_ && fabs(u_ch2_balance_.sensor.angle) < 4)
                {
                    u_ch2_balance_.param.angle_offset = u_ch2_balance_.sensor.angle;
                    printf("UNILATERA ALIBRATION succeeded !!!");
                    control_mode_ = eNORMAL;
                } 
                MotorStop();
            break;
        case eMOTOR_TEST:
                if(motor_test_start_)
                {
                    motor_set_speed_.ch1 = 0.5f;
                    motor_set_speed_.ch2 = 0.5f;
                    motor_set_speed_.ch3 = 0.5f;
                }
            break;
        default:
            MotorStop();
            break;
        }
    }
    else
    {
        MotorStop();
        if(low_power_reminder_time_.WaitMs(1000) &&
            systeam_low_power_ == true)
        {
            printf("low power!!\r\n");
        }
    }
    bsp_driver_.can_.CanSendMotorSpeed(motor_set_speed_.ch2, motor_set_speed_.ch3);
}

void CubliMiniControl::ModeControl()
{
    KeyAction_e key1_action = bsp_driver_.key1_.GetKeyAction();
    KeyAction_e key2_action = bsp_driver_.key2_.GetKeyAction();
    switch(control_mode_)
    {
        case eNORMAL:
        {
            bsp_driver_.green_led_.AlwaysOn();
            bsp_driver_.bule_led_.AlwaysOff();
            if(key2_action == eKEY_ACTION_SHORT_PRESS)
            {
                control_mode_ = ePOINT_BALANCE;
            }
        }
        break;
        case ePOINT_BALANCE:
            {
                bsp_driver_.green_led_.Control(200, 1, 2000);
                bsp_driver_.bule_led_.AlwaysOff();
                if(key2_action == eKEY_ACTION_SHORT_PRESS)
                {
                    control_mode_ = eUNILATERAL_BALANCE;
                }

                if(key1_action == eKEY_ACTION_LONG_PRESS)
                {
                    control_mode_ = ePOINT_CALIBRATION;
                }
            }
        break;
        case eUNILATERAL_BALANCE:
            {
                bsp_driver_.green_led_.Control(200, 2, 2000);
                bsp_driver_.bule_led_.AlwaysOff();
                if(key2_action == eKEY_ACTION_SHORT_PRESS)
                {
                    control_mode_ = eMOTOR_TEST;
                }

                if(key1_action == eKEY_ACTION_LONG_PRESS)
                {
                    control_mode_ = eUNILATERA_CALIBRATION;
                }
            }
        break;
        case ePOINT_CALIBRATION:
            bsp_driver_.green_led_.Control(200, 1, 2000);
            bsp_driver_.bule_led_.Control(200, 4, 1000);
            if(key1_action == eKEY_ACTION_LONG_PRESS)
            {
                control_mode_ = eNORMAL;
            }
            break;
        case eUNILATERA_CALIBRATION:
            bsp_driver_.green_led_.Control(200, 2, 2000);
            bsp_driver_.bule_led_.Control(200, 4, 1000);
            if(key1_action == eKEY_ACTION_LONG_PRESS)
            {
                control_mode_ = eNORMAL;
            }
            break;    
        case eMOTOR_TEST:
            bsp_driver_.green_led_.Control(200, 3, 2000);
            bsp_driver_.bule_led_.AlwaysOff();
            if(key2_action == eKEY_ACTION_SHORT_PRESS)
            {
                control_mode_ = eNORMAL;
                motor_test_start_ = false;
            }
            if(key1_action == eKEY_ACTION_LONG_PRESS)
            {
                motor_test_start_ = true;
            }
            break;
        default:
            bsp_driver_.green_led_.AlwaysOn();
            bsp_driver_.bule_led_.AlwaysOff();
            control_mode_ = eNORMAL;
            break;
    }

    if(bsp_driver_.GetBatVoltage() < LOW_POWER_VOLTAGE_THRESHOLD)
    {
        bsp_driver_.bule_led_.AlwaysOn();
        systeam_low_power_ = true;
        control_mode_ = eNORMAL;
    }
    else
    {
        systeam_low_power_ = false;
    }

    uint8_t temp_status_ = (uint8_t)bsp_driver_.can_.can_is_online_ << 1 | motor_init_status_.ch1;
    switch(temp_status_)
    {
        case eDEVICE_ONLINE:
            mcu1_device_healt_ = DeviceHealt_e::eDEVICE_ONLINE;
            bsp_driver_.motor_green_led_.Control(200, 1, 3000);
            break;
        case eDEVICE_MOTOR1_OFFLINE:
            mcu1_device_healt_ = DeviceHealt_e::eDEVICE_MOTOR1_OFFLINE;
            bsp_driver_.motor_green_led_.Control(200, 2, 3000);
            break;
        case eDEVICE_CAN_OFFLINE:
            mcu1_device_healt_ = DeviceHealt_e::eDEVICE_CAN_OFFLINE;
            bsp_driver_.motor_green_led_.Control(200, 3, 3000);
            break;
        case eDEVICE_ALL_OFFLINE:
            mcu1_device_healt_ = DeviceHealt_e::eDEVICE_ALL_OFFLINE;
            bsp_driver_.motor_green_led_.Control(200, 4, 3000);
            break;
        default:
            mcu1_device_healt_ = DeviceHealt_e::eDEVICE_ALL_OFFLINE;
            bsp_driver_.motor_green_led_.Control(200, 4, 3000);
            break;
    }
}

void CubliMiniControl::SensorUpdate()
{
    if(ahrs_->ImuUpdate())
    {
        p_balance_.sensor.x.angle = ahrs_->imu_data_.angle.pitch;
        p_balance_.sensor.x.gyro = ahrs_->imu_data_.gyro.gy;

        p_balance_.sensor.y.angle = ahrs_->imu_data_.angle.roll;
        p_balance_.sensor.y.gyro = ahrs_->imu_data_.gyro.gx;

        p_balance_.sensor.z.angle = ahrs_->imu_data_.angle.yaw;
        p_balance_.sensor.z.gyro = ahrs_->imu_data_.gyro.gz;

        u_ch2_balance_.sensor.angle = ahrs_->imu_data_.angle.pitch / cos(35.3f / 57.3f);
        u_ch2_balance_.sensor.gyro = ahrs_->imu_data_.gyro.gy / cos(35.3f / 57.3f);
    }

    bsp_driver_.can_.CanGetMsg(motor_get_speed_.ch2, motor_get_speed_.ch3);
}

void CubliMiniControl::SetCh1MotorInitResult(uint8_t _result)
{
    motor_init_status_.ch1 = _result;
}

void CubliMiniControl::FirstWriteParamToEeprom()
{
    PBalanceParamInit();
    Ch2AxisUBalanceParamInit();
}

void CubliMiniControl::PBalanceParamInit()
{
    p_balance_.param.x.angle_offset = 0.5f;
    p_balance_.param.x.kp = -17;
    p_balance_.param.x.kv = 1.5f;
    p_balance_.param.x.ks = 0.3f;

    p_balance_.param.y.angle_offset = 1.5f;
    p_balance_.param.y.kp = 18;
    p_balance_.param.y.kv = 1.6f;
    p_balance_.param.y.ks = 0.2f;

    p_balance_.param.z.angle_offset = 0;
    p_balance_.param.z.kp = 0;
    p_balance_.param.z.kv = -0.3f;
    p_balance_.param.z.ks = 0.02f;

    p_balance_.param.axis_offset = 0;

    bool s = SavePBalanceParam(p_balance_.param);
    printf("save p param :%d\r\n", s);
}

void CubliMiniControl::Ch2AxisUBalanceParamInit()
{
    u_ch2_balance_.param.kp = -25;
    u_ch2_balance_.param.kv = 2;
    u_ch2_balance_.param.ks = 0.5f;
    u_ch2_balance_.param.angle_offset = 0;

    bool s = SaveUBalanceParam(u_ch2_balance_.param);
    printf("save u param :%d\r\n", s);
}


void CubliMiniControl::MotorStop()
{
    motor_set_speed_.ch1 = 0;
    motor_set_speed_.ch2 = 0;
    motor_set_speed_.ch3 = 0;
}

float CubliMiniControl::Limit(float _data, float _limit)
{
    if(_data >= _limit)
    {
        _data = _limit;
    }
    if(_data <= -_limit)
    {
        _data = -_limit;
    }
    return _data;
}

} // namespace Cubli 
} // namespace Control 