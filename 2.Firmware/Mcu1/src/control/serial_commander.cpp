
#include "control/serial_commander.h"

namespace CubliMini {
namespace Control {

void SerialCommander::cmd_printf(const char *fmt, ...)
{
    va_list args;                  // 定义一个va_list类型的变量，用来储存单个参数
    va_start(args, fmt);           // 使args指向可变参数的第一个参数
    int len = vprintf(fmt, args);  // 必须用vcmd_printf等带V的
    va_end(args);                  // 结束可变参数的获取
}

uint16_t SerialCommander::GetCharLen(const char *user_cmd)
{
    return strlen(user_cmd) - 2;  // - \r\n
}

void SerialCommander::GetOrSet(const char *user_cmd, CubliMiniControl &control)
{
    if (GetCharLen(user_cmd) < 1)
    {
        cmd_printf("len < 1\r\n");
        return;
    }

    int value;
    char cmd1 = user_cmd[0];
    switch (cmd1)
    {
    case SCMD_BYTE_1_G:
        // get cmd
        GetCmd(user_cmd, control);
        break;
    case SCMD_BYTE_1_S:
        // set cmd
        SetCmd(user_cmd, control);
        break;
    case SCMD_BYTE_1_C:
        Calibration(user_cmd, control);
        break;
    case SCMD_BYTE_1_F:
        value = atoi(&user_cmd[1]);
        if (value == 1)
        {
            cmd_printf("defalut param\r\n");
            control.FirstWriteParamToEeprom();
        }
        else
        {
            cmd_printf("byte2 error\r\n");
        }
        break;
    case SCMD_BYTE_1_R:
        control.dynamic_param_.write_once_default_param = true;
        // esp_restart();
        break;
    default:
        cmd_printf("byte1 error\r\n");
        break;
    }
}

void SerialCommander::Calibration(const char *user_cmd, CubliMiniControl &control)
{
    if (GetCharLen(user_cmd) < 2)
    {
        cmd_printf("len < 2\r\n");
        return;
    }

    char cmd2 = user_cmd[1];
    switch (cmd2)
    {
    case SCMD_BYTE_2_P:
        cmd_printf("start calibration p\r\n");
        control.StartPBalanceAutoCalibration();
        break;
    case SCMD_BYTE_2_U:
        cmd_printf("start calibration u\r\n");
        control.StartUBalanceAutoCalibration();
        break;
    default:
        cmd_printf("byte2 error\r\n");
        break;
    };
}

void SerialCommander::GetCmd(const char *user_cmd, CubliMiniControl &control)
{
    if (GetCharLen(user_cmd) < 2)
    {
        cmd_printf("len < 2\r\n");
        return;
    }
    char cmd2 = user_cmd[1];

    PBalanceParam_t _p_parm = control.p_balance_.param;
    AxisParam_t _u_param    = control.u_ch2_balance_.param;
    switch (cmd2)
    {
    case SCMD_BYTE_2_P:
        // cmd_printf p param
        cmd_printf(
            "p param x_kp:%f x_kv:%f x_ks:%f x_A:%f y_kp:%f y_kv:%f y_ks:%f y_A:%f z_kp:%f z_kv:%f "
            "z_ks:%f z_A:%f axis:%f\r\n",
            _p_parm.x.kp,
            _p_parm.x.kv,
            _p_parm.x.ks,
            _p_parm.x.angle_offset,
            _p_parm.y.kp,
            _p_parm.y.kv,
            _p_parm.y.ks,
            _p_parm.y.angle_offset,
            _p_parm.z.kp,
            _p_parm.z.kv,
            _p_parm.z.ks,
            _p_parm.z.angle_offset,
            _p_parm.axis_offset);
        break;
    case SCMD_BYTE_2_U:
        cmd_printf(
            "u param kp:%f kv:%f ks:%f A:%f\r\n",
            _u_param.kp,
            _u_param.kv,
            _u_param.ks,
            _u_param.angle_offset);
        // cmd_printf p param
        break;
    case SCMD_BYTE_2_W:
    {
        cmd_printf(
            "WifiParam: use_wifi: %d ssid: %s password: %s server_port: %d ssid_len: %d "
            "password_len: %d\r\n",
            control.wifi_param_.use_wifi,
            control.wifi_param_.wifi_ssid,
            control.wifi_param_.wifi_password,
            control.wifi_param_.server_port,
            control.wifi_param_.wifi_ssid_len,
            control.wifi_param_.wifi_password_len);
    }
    break;
    case SCMD_BYTE_2_A:
    {
        output_angle_ = true;
    }
    break;
    default:
        cmd_printf("byte2 error\r\n");
        break;
    }
}

void SerialCommander::SetCmd(const char *user_cmd, CubliMiniControl &control)
{
    if (GetCharLen(user_cmd) < 2)
    {
        cmd_printf("len < 2\r\n");
        return;
    }
    char cmd2 = user_cmd[1];
    switch (cmd2)
    {
    case SCMD_BYTE_2_P:
        SetPParam(user_cmd, control.p_balance_.param);
        break;
    case SCMD_BYTE_2_U:
        SetUParam(user_cmd, control.u_ch2_balance_.param);
        break;
    case SCMD_BYTE_2_S:
        SaveParam(user_cmd, control.p_balance_.param, control.u_ch2_balance_.param);
        break;
    case SCMD_BYTE_2_W:
        SaveWifiParam(user_cmd, control);
        break;
    default:
        cmd_printf("byte2 error\r\n");
        break;
    }
}

void SerialCommander::SaveWifiParam(const char *user_cmd, CubliMiniControl &control)
{
    if (GetCharLen(user_cmd) < 3)
    {
        cmd_printf("len < 4\r\n");
        return;
    }
    char cmd3 = user_cmd[2];
    char cmd4 = user_cmd[3];
    switch (cmd3)
    {
    case SCMD_BYTE_3_E:
    {
        int value = atoi(&user_cmd[3]);
        if (value == 0)
        {
            control.wifi_param_.use_wifi = false;
            bool res                     = SaveWiflParam(control.wifi_param_);
            cmd_printf("close wifi, save res: %d\r\n", res);
        }
        else if (value == 1)
        {
            control.wifi_param_.use_wifi = true;
            bool res                     = SaveWiflParam(control.wifi_param_);
            cmd_printf("open wifi, save res: %d\r\n", res);
        }
        else
        {
            cmd_printf("byte4 error\r\n");
        }
    }
    break;
    case SCMD_BYTE_3_S:
        if (cmd4 == ':')
        {
            // 3 + 2是为了删除"\r\n", memset已经补0
            int len = strlen(user_cmd) - 5 < WIFI_CHAR_LEN ? strlen(user_cmd) - 5 : WIFI_CHAR_LEN;
            control.wifi_param_.wifi_ssid_len = len;
            memset(control.wifi_param_.wifi_ssid, 0, WIFI_CHAR_LEN);
            memcpy(control.wifi_param_.wifi_ssid, &user_cmd[4], len);
            bool res = SaveWiflParam(control.wifi_param_);
            cmd_printf("ssid: %s save res: %d\r\n", control.wifi_param_.wifi_ssid, res);
        }
        else
        {
            cmd_printf("byte4 error\r\n");
        }
        break;
    case SCMD_BYTE_3_P:
        if (cmd4 == ':')
        {
            int len = strlen(user_cmd) - 5 < WIFI_CHAR_LEN ? strlen(user_cmd) - 5 : WIFI_CHAR_LEN;
            control.wifi_param_.wifi_password_len = len;
            memset(control.wifi_param_.wifi_password, 0, WIFI_CHAR_LEN);
            memcpy(control.wifi_param_.wifi_password, &user_cmd[4], len);
            bool res = SaveWiflParam(control.wifi_param_);
            cmd_printf("password: %s save res: %d\r\n", control.wifi_param_.wifi_password, res);
        }
        else
        {
            cmd_printf("byte4 error\r\n");
        }
        break;
    default:
        cmd_printf("byte3 error\r\n");
        break;
    }
}

void SerialCommander::SaveParam(
    const char *user_cmd, PBalanceParam_t &_p_parm, AxisParam_t &_u_param)
{
    if (GetCharLen(user_cmd) < 3)
    {
        cmd_printf("len < 3\r\n");
        return;
    }
    char cmd3 = user_cmd[2];
    switch (cmd3)
    {
    case SCMD_BYTE_3_P:
        SavePBalanceParam(_p_parm);
        cmd_printf("save p parm\r\n");
        cmd_printf(
            "p param x_kp:%f x_kv:%f x_ks:%f x_A:%f y_kp:%f y_kv:%f y_ks:%f y_A:%f z_kp:%f z_kv:%f "
            "z_ks:%f z_A:%f axis:%f\r\n",
            _p_parm.x.kp,
            _p_parm.x.kv,
            _p_parm.x.ks,
            _p_parm.x.angle_offset,
            _p_parm.y.kp,
            _p_parm.y.kv,
            _p_parm.y.ks,
            _p_parm.y.angle_offset,
            _p_parm.z.kp,
            _p_parm.z.kv,
            _p_parm.z.ks,
            _p_parm.z.angle_offset,
            _p_parm.axis_offset);
        break;
    case SCMD_BYTE_3_U:
        SaveUBalanceParam(_u_param);
        cmd_printf("save u parm\r\n");
        cmd_printf(
            "u param kp:%f kv:%f ks:%f A:%f\r\n",
            _u_param.kp,
            _u_param.kv,
            _u_param.ks,
            _u_param.angle_offset);
        break;
    default:
        cmd_printf("byte3 error\r\n");
        break;
    }
}

void SerialCommander::SetPParam(const char *user_cmd, PBalanceParam_t &_param)
{
    if (GetCharLen(user_cmd) < 4)
    {
        cmd_printf("len < 4\r\n");
        return;
    }
    char cmd3   = user_cmd[2];
    float value = atof(&user_cmd[3]);
    switch (cmd3)
    {
    case SCMD_BYTE_3_X:
        SetAxisParam(user_cmd, _param.x);
        break;
    case SCMD_BYTE_3_Y:
        SetAxisParam(user_cmd, _param.y);
        break;
    case SCMD_BYTE_3_Z:
        SetAxisParam(user_cmd, _param.z);
        break;
    case SCMD_BYTE_3_A:
        _param.axis_offset = value;
        cmd_printf("axis: %f\r\n", _param.axis_offset);
        break;
    default:
        cmd_printf("byte3 error\r\n");
        break;
    }
}

void SerialCommander::SetUParam(const char *user_cmd, AxisParam_t &_param)
{
    if (GetCharLen(user_cmd) < 4)
    {
        cmd_printf("len < 4\r\n");
        return;
    }
    char cmd3   = user_cmd[2];
    float value = atof(&user_cmd[3]);
    switch (cmd3)
    {
    case SCMD_BYTE_3_P:
        _param.kp = value;
        cmd_printf("kp: %f\r\n", value);
        break;
    case SCMD_BYTE_3_V:
        _param.kv = value;
        cmd_printf("kv: %f\r\n", value);
        break;
    case SCMD_BYTE_3_S:
        _param.ks = value;
        cmd_printf("ks: %f\r\n", value);
        break;
    case SCMD_BYTE_3_A:
        _param.angle_offset = value;
        cmd_printf("A: %f\r\n", value);
        break;
    default:
        cmd_printf("byte4 error\r\n");
        break;
    }
}

void SerialCommander::SetAxisParam(const char *user_cmd, AxisParam_t &_param)
{
    if (GetCharLen(user_cmd) < 5)
    {
        cmd_printf("len < 5\r\n");
        return;
    }
    char cmd4   = user_cmd[3];
    float value = atof(&user_cmd[4]);
    switch (cmd4)
    {
    case SCMD_BYTE_4_P:
        _param.kp = value;
        cmd_printf("kp: %f\r\n", value);
        break;
    case SCMD_BYTE_4_V:
        _param.kv = value;
        cmd_printf("kv: %f\r\n", value);
        break;
    case SCMD_BYTE_4_S:
        _param.ks = value;
        cmd_printf("ks: %f\r\n", value);
        break;
    case SCMD_BYTE_4_A:
        _param.angle_offset = value;
        cmd_printf("A: %f\r\n", value);
        break;
    default:
        cmd_printf("byte4 error\r\n");
        break;
    }
}

bool SerialCommander::isSentinel(char ch)
{
    if (ch == keol)
    {
        return true;
    }
    else if (ch == '\r')
    {
        cmd_printf("Warn: \\r detected! \n");
    }
    return false;
}

void SerialCommander::Run(Stream &_serial, CubliMiniControl &control)
{
    if (_serial.available() > 0)
    {
        int ch                    = _serial.read();
        received_chars_[rec_cnt_] = (char)ch;
        rec_cnt_++;
        if (rec_cnt_ >= MAX_RECEIVER_SIZE)
        {
            rec_cnt_           = 0;
            received_chars_[0] = 0;
        }
        if (isSentinel(ch))
        {
            GetOrSet(received_chars_, control);
            received_chars_[0] = 0;
            rec_cnt_           = 0;
        }
    }
}

}  // namespace Control
}  // namespace CubliMini