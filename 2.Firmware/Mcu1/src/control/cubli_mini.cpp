#include "control/cubli_mini.h"

namespace CubliMini {
namespace Control {

void CubliMiniControl::LinkSensor(AHRS *_ahrs)
{
    ahrs_ = _ahrs;
}

void CubliMiniControl::FirstDownloadFlash()
{
    EepromInit();
    uint32_t flag = 0xffffffff;
    ReadFirstDownload(flag);
    if(flag != 0)
    {
        flag = 0;
        SaveFirstDownload(flag);
        FirstWriteParamToEeprom();
        printf("first write\r\n");
    }
}

void CubliMiniControl::Init()
{
    z_speed_ = 0;
    pbalance_calibration_mode_ = eEND;
    start_pbalance_calibration_ = false;

    ubalance_calibration_mode_ = eEND;
    start_ubalance_calibration_ = false;

    memset(&dynamic_param_, 0, sizeof(dynamic_param_));

    FirstDownloadFlash();

    ParamInit(p_balance_.param, u_ch2_balance_.param, wifi_param_);

    printf("WifiParam: use_wifi: %d ssid: %s password: %s server_port: %d ssid_len: %d password_len: %d\r\n", 
        wifi_param_.use_wifi,
        wifi_param_.wifi_ssid,
        wifi_param_.wifi_password,
        wifi_param_.server_port,
        wifi_param_.wifi_ssid_len,
        wifi_param_.wifi_password_len);

    control_mode_ = eNORMAL;
    cube_is_upside_dowm_ = true;
    motor_test_start_ = false;
    systeam_low_power_ = false;
    protection_threshold_ = BALABCE_PROTECTION_THRESHOLD;
    mcu1_device_healt_ = eDEVICE_ALL_OFFLINE;
    p_balance_.param.z.angle_offset = 0;
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

void CubliMiniControl::GetMotorMoveSpeed(float _x_speed, 
                                         float _y_speed, 
                                         float _z_speed, 
                                         MotorSetSpeed_t & _set_speed)
{
    _set_speed.ch1 = - _y_speed * cos((30) / 57.3f) - _x_speed * sin((30) / 57.3f) + _z_speed / 3.0f + z_speed_;
    _set_speed.ch2 = _x_speed + _z_speed  / 3.0f + z_speed_;
    _set_speed.ch3 =  -_x_speed * sin((30) / 57.3f) + _y_speed * cos((30) / 57.3f) + _z_speed / 3.0f + z_speed_;
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
    motor_set_speed_.ch2 = -Limit(motor_set_speed_.ch2, VOLTAGE_LIMIT);
}

void CubliMiniControl::PBalance()
{
    if(ahrs_->is_static_)
    {
        cube_is_upside_dowm_ = false;
        // p_balance_.param.z.angle_offset = p_balance_.sensor.z.angle;
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

    motor_set_speed_.ch1 = -Limit(motor_set_speed_.ch1, VOLTAGE_LIMIT);
    motor_set_speed_.ch2 = -Limit(motor_set_speed_.ch2, VOLTAGE_LIMIT);
    motor_set_speed_.ch3 = -Limit(motor_set_speed_.ch3, VOLTAGE_LIMIT);
}

void CubliMiniControl::PBalanceCalibrationAngle_X(CalibrationMode_e & mode)
{
    if(pbalance_timeout_x_.WaitMs(AUTO_P_CALIBRATION_X_TIMEOUT))
    {
        mode = eY_AXIS;
        printf("PBalance: X-axis calibration timeout\r\n");
        return;
    }

    const float angle_scale = AUTO_CALIBRATION_ANGLE_SCALE;
    float get_speed = motor_get_speed_.ch2;

    if(fabs(get_speed) >= AUTO_CALIBRATION_SPEED_THRESHOLD)
    {
        if(get_speed >= 0)
        {
            p_balance_.param.x.angle_offset += angle_scale;
        }
        else
        {
            p_balance_.param.x.angle_offset -= angle_scale;
        }
    }
    else
    {
        mode = eY_AXIS;
        printf("PBalance: X-axis calibration completed\r\n");
    }
}

#define SAMPLE_TIME 0.01   // 采样时间，单位为秒
#define ANGLE_LIMIT 10.0   // 角度限幅，单位为度
#define ANGLE_LIMIT_RAD (ANGLE_LIMIT * M_PI / 180.0)   // 角度限幅，单位为弧度
#define INTEGRAL_LIMIT 0.01  // 积分限幅，单位为弧度

double integrate(double omega, double *angle, double *error, double *integral) 
{
    // 计算误差
    double e = *error - *angle;

    // 积分计算
    *integral += e * SAMPLE_TIME;

    // 积分限幅处理
    if (*integral > INTEGRAL_LIMIT) {
        *integral = INTEGRAL_LIMIT;
    }
    else if (*integral < -INTEGRAL_LIMIT) {
        *integral = -INTEGRAL_LIMIT;
    }

    // 角度积分计算
    double angle_increment = (*integral + e) * SAMPLE_TIME;
    *angle += angle_increment;

    // 角度限幅处理
    if (*angle > ANGLE_LIMIT_RAD) {
        *angle = ANGLE_LIMIT_RAD;
    }
    else if (*angle < -ANGLE_LIMIT_RAD) {
        *angle = -ANGLE_LIMIT_RAD;
    }

    // 更新误差值
    *error = *angle;

    // 返回当前角度值（单位：弧度）
    return *angle;
}

void CubliMiniControl::Autorotation()
{

}

void CubliMiniControl::PBalanceCalibrationAngle_Y(CalibrationMode_e & mode)
{
    if(pbalance_timeout_y_.WaitMs(AUTO_P_CALIBRATION_Y_TIMEOUT))
    {
        mode = eEND;
        printf("PBalance: Y-axis calibration timeout\r\n");
        return;
    }

    const float angle_scale = AUTO_CALIBRATION_ANGLE_SCALE;
    float get_speed = motor_get_speed_.ch1 + motor_get_speed_.ch3;
    float get_angle = p_balance_.sensor.x.angle;

    if( fabs(get_speed) >= AUTO_CALIBRATION_SPEED_THRESHOLD || 
        fabs(motor_get_speed_.ch1 >= AUTO_CALIBRATION_SPEED_THRESHOLD) || 
        fabs(motor_get_speed_.ch3 >= AUTO_CALIBRATION_SPEED_THRESHOLD))
    {
        if( motor_get_speed_.ch1 >= AUTO_CALIBRATION_SPEED_THRESHOLD && 
            motor_get_speed_.ch3 <= -AUTO_CALIBRATION_SPEED_THRESHOLD)
        {
            p_balance_.param.y.angle_offset += angle_scale;
        }
        else if(motor_get_speed_.ch1 <= -AUTO_CALIBRATION_SPEED_THRESHOLD && 
                motor_get_speed_.ch3 >= AUTO_CALIBRATION_SPEED_THRESHOLD)
        {
            p_balance_.param.y.angle_offset -= angle_scale;
        }
        else if(fabs(motor_get_speed_.ch1) <= AUTO_CALIBRATION_SPEED_THRESHOLD && 
                motor_get_speed_.ch3 >= AUTO_CALIBRATION_SPEED_THRESHOLD)
        {
            p_balance_.param.y.angle_offset -= angle_scale;
        }
        else if(fabs(motor_get_speed_.ch1) <= AUTO_CALIBRATION_SPEED_THRESHOLD && 
                motor_get_speed_.ch3 <= -AUTO_CALIBRATION_SPEED_THRESHOLD)
        {
            p_balance_.param.y.angle_offset += angle_scale;
        }
        else if(motor_get_speed_.ch1 >= AUTO_CALIBRATION_SPEED_THRESHOLD && 
                fabs(motor_get_speed_.ch3 <= AUTO_CALIBRATION_SPEED_THRESHOLD))
        {
            p_balance_.param.y.angle_offset += angle_scale;
        }
        else if(motor_get_speed_.ch1 <= -AUTO_CALIBRATION_SPEED_THRESHOLD && 
                fabs(motor_get_speed_.ch3 <= -AUTO_CALIBRATION_SPEED_THRESHOLD))
        {
            p_balance_.param.y.angle_offset -= angle_scale;
        }
    }
    else if(fabs(get_speed) <= AUTO_CALIBRATION_SPEED_THRESHOLD && 
            fabs(motor_get_speed_.ch1 <= AUTO_CALIBRATION_SPEED_THRESHOLD) && 
            fabs(motor_get_speed_.ch3 <= AUTO_CALIBRATION_SPEED_THRESHOLD))
    {
        mode = eEND;
        printf("PBalance: Y-axis calibration completed\r\n");
    }
}

void CubliMiniControl::StartPBalanceAutoCalibration()
{
    start_pbalance_calibration_ = true;
    pbalance_calibration_mode_ = eX_AXIS;
}

void CubliMiniControl::PBalanceAutoCalibration()
{
    if(start_pbalance_calibration_ == false)
    {
        return;
    }

    if(!pbalance_calibration_period_.WaitMs(AUTO_P_CALIBRATION_PERIOD))
    {
        return;
    }

    switch (pbalance_calibration_mode_)
    {
        case eX_AXIS:
        {
            PBalanceCalibrationAngle_X(pbalance_calibration_mode_);
        }
        break;
        case eY_AXIS:
        {
            PBalanceCalibrationAngle_Y(pbalance_calibration_mode_);
        }
        break;
        case eEND:
        {
            start_pbalance_calibration_ = false;
            pbalance_calibration_mode_ = eX_AXIS;
            pbalance_timeout_y_.ResetWaitMs();
            pbalance_timeout_x_.ResetWaitMs();
            pbalance_calibration_period_.ResetWaitMs();
            control_mode_ = ePOINT_BALANCE;
            SavePBalanceParam(p_balance_.param);
            printf("PBalance: calibration completed\r\n");
        }
        break;
    }
}

void CubliMiniControl::UBalanceCalibrationAngle_X(CalibrationMode_e & mode)
{
    if(ubalance_timeout_x_.WaitMs(AUTO_P_CALIBRATION_X_TIMEOUT))
    {
        mode = eEND;
        printf("UBalance: X-axis calibration timeout\r\n");
        return;
    }

    const float angle_scale = AUTO_CALIBRATION_ANGLE_SCALE;
    float get_speed = u_ch2_balance_.sensor.speed;
    // 转速为6，经验值，旋转比较缓慢
    if(fabs(get_speed) >= AUTO_CALIBRATION_SPEED_THRESHOLD)
    {
        if(get_speed >= 0)
        {
            u_ch2_balance_.param.angle_offset += angle_scale;
        }
        else
        {
            u_ch2_balance_.param.angle_offset -= angle_scale;
        }
    }
    else
    {
        mode = eEND;
        printf("PBalance: X-axis calibration completed\r\n");
    }
}

void CubliMiniControl::StartUBalanceAutoCalibration()
{
    start_ubalance_calibration_ = true;
    ubalance_calibration_mode_ = eX_AXIS;
}

void CubliMiniControl::UBalanceAutoCalibration()
{
    if(start_ubalance_calibration_ == false)
    {
        return;
    }

    if(!ubalance_calibration_period_.WaitMs(AUTO_U_CALIBRATION_PERIOD))
    {
        return;
    }

    switch (ubalance_calibration_mode_)
    {
        case eX_AXIS:
        {
            UBalanceCalibrationAngle_X(ubalance_calibration_mode_);
        }
        break;
        case eEND:
        {
            start_ubalance_calibration_ = false;
            ubalance_calibration_mode_ = eX_AXIS;
            ubalance_timeout_x_.ResetWaitMs();
            ubalance_calibration_period_.ResetWaitMs();
            control_mode_ = eUNILATERA_BALANCE;
            SaveUBalanceParam(u_ch2_balance_.param);
            printf("UBalance: calibration completed\r\n");
        }
        break;
    }
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
        case eUNILATERA_BALANCE:
                Ch2AxisUBalance();
            break;
        case ePOINT_CALIBRATION:
                PBalance();
            break;
        case ePOINT_CALIBRATIONING:
                PBalance();
            break;
        case eUNILATERA_CALIBRATION:
                Ch2AxisUBalance();
            break;
        case eUNILATERA_CALIBRATIONING:
                Ch2AxisUBalance();
            break;
        case eMOTOR_TEST:
                if(motor_test_start_)
                {
                    motor_set_speed_.ch1 = 0.7f;
                    motor_set_speed_.ch2 = 0.7f;
                    motor_set_speed_.ch3 = 0.7f;
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
                    control_mode_ = eUNILATERA_BALANCE;
                }

                if(key1_action == eKEY_ACTION_LONG_PRESS)
                {
                    control_mode_ = ePOINT_CALIBRATION;
                }
            }
        break;
        case eUNILATERA_BALANCE:
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
            if( fabs(p_balance_.sensor.x.angle - p_balance_.param.x.angle_offset) < AUTO_CALIBRATION_STATIC_THRESHOLD &&  
                fabs(p_balance_.sensor.y.angle - p_balance_.param.y.angle_offset) < AUTO_CALIBRATION_STATIC_THRESHOLD)
            {
                // 3s
                if(wait_p_calibration_.WaitMs(AUTO_P_CALIBRATION_WAIT_STATIC))
                {
                    StartPBalanceAutoCalibration();
                    control_mode_ = ePOINT_CALIBRATIONING;
                    wait_p_calibration_.ResetWaitMs();
                }
            }
            else
            {
                wait_p_calibration_.ResetWaitMs();
            }

            bsp_driver_.green_led_.Control(200, 1, 2000);
            bsp_driver_.bule_led_.Control(200, 1, 1000);

            if(key1_action == eKEY_ACTION_LONG_PRESS)
            {
                control_mode_ = eNORMAL;
            }
            break;
        case ePOINT_CALIBRATIONING:
            bsp_driver_.green_led_.Control(200, 1, 2000);
            bsp_driver_.bule_led_.Control(200, 5, 1000);

            if(key1_action == eKEY_ACTION_LONG_PRESS)
            {
                control_mode_ = eNORMAL;
            }
            break;
        case eUNILATERA_CALIBRATION:
            if( fabs(u_ch2_balance_.sensor.angle - u_ch2_balance_.param.angle_offset) < AUTO_CALIBRATION_STATIC_THRESHOLD)
            {
                if(wait_u_calibration_.WaitMs(AUTO_U_CALIBRATION_WAIT_STATIC))
                {
                    StartUBalanceAutoCalibration();
                    control_mode_ = eUNILATERA_CALIBRATIONING;
                    wait_u_calibration_.ResetWaitMs();
                }
            }
            else
            {
                wait_u_calibration_.ResetWaitMs();
            }
            bsp_driver_.green_led_.Control(200, 2, 2000);
            bsp_driver_.bule_led_.Control(200, 1, 1000);
            if(key1_action == eKEY_ACTION_LONG_PRESS)
            {
                control_mode_ = eNORMAL;
            }
            break;
        case eUNILATERA_CALIBRATIONING:
            bsp_driver_.green_led_.Control(200, 2, 2000);
            bsp_driver_.bule_led_.Control(200, 5, 1000);

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

        // p_balance_.sensor.z.angle = ahrs_->imu_data_.angle.yaw;
        // p_balance_.sensor.z.angle = 0;
        p_balance_.sensor.z.gyro = ahrs_->imu_data_.gyro.gz;

        u_ch2_balance_.sensor.angle = ahrs_->imu_data_.angle.pitch / cos(35.3f / 57.3f);
        u_ch2_balance_.sensor.gyro = ahrs_->imu_data_.gyro.gy / cos(35.3f / 57.3f);
    }

    bsp_driver_.can_.CanGetMsg(motor_get_speed_.ch2, motor_get_speed_.ch3);
}

void CubliMiniControl::ResetCanQueue()
{
    bsp_driver_.can_.ResetQueue();
}

void CubliMiniControl::SetCh1MotorInitResult(uint8_t _result)
{
    motor_init_status_.ch1 = _result;
}

void CubliMiniControl::FirstWriteParamToEeprom()
{
    PBalanceWriteDefaultParam();
    Ch2AxisUBalanceWriteDefaultParam();
    WifiWriteDefaultParam();
}

void CubliMiniControl::PBalanceWriteDefaultParam()
{
    p_balance_.param.x.angle_offset = P_BALANCE_X_A;
    p_balance_.param.x.kp = P_BALANCE_X_P;
    p_balance_.param.x.kv = P_BALANCE_X_V;
    p_balance_.param.x.ks = P_BALANCE_X_S;

    p_balance_.param.y.angle_offset = P_BALANCE_Y_A;
    p_balance_.param.y.kp = P_BALANCE_Y_P;
    p_balance_.param.y.kv = P_BALANCE_Y_V;
    p_balance_.param.y.ks = P_BALANCE_Y_S;

    p_balance_.param.z.angle_offset = P_BALANCE_Z_A;
    p_balance_.param.z.kp = P_BALANCE_Z_P;
    p_balance_.param.z.kv = P_BALANCE_Z_V;
    p_balance_.param.z.ks = P_BALANCE_Z_S;

    p_balance_.param.axis_offset = 0;

    bool s = SavePBalanceParam(p_balance_.param);
    printf("save p param :%d\r\n", s);
}

void CubliMiniControl::Ch2AxisUBalanceWriteDefaultParam()
{
    u_ch2_balance_.param.kp = U_BALANCE_CH2_P;
    u_ch2_balance_.param.kv = U_BALANCE_CH2_V;
    u_ch2_balance_.param.ks = U_BALANCE_CH2_S;
    u_ch2_balance_.param.angle_offset = U_BALANCE_CH2_A;

    bool s = SaveUBalanceParam(u_ch2_balance_.param);
    printf("save u param :%d\r\n", s);
}

void CubliMiniControl::WifiWriteDefaultParam()
{
    wifi_param_.use_wifi = false;
    const char * ssid = WIFI_DEFAULT_SSID;
    wifi_param_.wifi_ssid_len = strlen(ssid) < WIFI_CHAR_LEN ? strlen(ssid) : WIFI_CHAR_LEN;
    memcpy(wifi_param_.wifi_ssid, ssid, wifi_param_.wifi_ssid_len);
    const char * passwork = WIFI_DEFAULT_PASSWORD;
    wifi_param_.wifi_password_len = strlen(passwork) < WIFI_CHAR_LEN ? strlen(passwork) : WIFI_CHAR_LEN;
    memcpy(wifi_param_.wifi_password, passwork, wifi_param_.wifi_password_len);
    wifi_param_.server_port = WIFI_DEFAULT_TCP_SERVER_PORT;
    bool res = SaveWiflParam(wifi_param_);
    printf("save wifi param :%d\r\n", res);
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