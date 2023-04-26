#include "control/param.h"

void ParamInit(PBalanceParam_t &_p_param, AxisParam_t & _u_param, WifiParam_t & _wifi_param)
{
    ReadPBalanceParam(_p_param);
    ReadUBalanceParam(_u_param);
    ReadWifiParam(_wifi_param);
}

bool SaveFirstDownload(uint32_t first)
{
    return Write(FIRST_DOWNLOAD_ADDR, (uint8_t *)&first, sizeof(first));
}

void ReadFirstDownload(uint32_t & first)
{
    Read(FIRST_DOWNLOAD_ADDR, (uint8_t *)&first, sizeof(first));
}

bool SavePBalanceParam(const PBalanceParam_t & _param)
{
    return Write(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void ReadPBalanceParam(PBalanceParam_t & _param)
{
    Read(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool SaveUBalanceParam(const AxisParam_t & _param)
{
    return Write(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void ReadUBalanceParam(AxisParam_t & _param)
{
    Read(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool SaveWiflParam(const WifiParam_t & _param)
{
    WifiParam_t temp;
    memset(&temp, 0, sizeof(temp));

    temp.use_wifi = _param.use_wifi;
    temp.server_port = _param.server_port;
    temp.wifi_ssid_len = _param.wifi_ssid_len;
    temp.wifi_password_len = _param.wifi_password_len;

    memcpy(temp.wifi_ssid, _param.wifi_ssid, _param.wifi_ssid_len);
    memcpy(temp.wifi_password, _param.wifi_password, _param.wifi_password_len);

    return Write(WIFI_PARAM_ADDR, (uint8_t *)&temp, sizeof(temp));
}

void ReadWifiParam(WifiParam_t & _param)
{
    WifiParam_t temp;
    Read(WIFI_PARAM_ADDR, (uint8_t *)&temp, sizeof(temp));

    memset(&_param, 0, sizeof(_param));

    _param.use_wifi = temp.use_wifi;
    _param.server_port = temp.server_port;
    _param.wifi_ssid_len = temp.wifi_ssid_len;
    _param.wifi_password_len = temp.wifi_password_len;

    memcpy(_param.wifi_ssid, temp.wifi_ssid, _param.wifi_ssid_len);
    memcpy(_param.wifi_password, temp.wifi_password, _param.wifi_password_len);
}
