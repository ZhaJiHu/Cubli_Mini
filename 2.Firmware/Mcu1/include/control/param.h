#pragma once
#include "bsp/my_eeprom.h"

struct AxisParam_t
{
    float kp;
    float kv;
    float ks;
    float angle_offset;
} ;

struct AxisSensor_t
{
    float gyro;
    float angle;
    float speed;
} ;

struct PBalanceParam_t
{
    AxisParam_t x;
    AxisParam_t y;
    AxisParam_t z;
    float axis_offset;
} ;

struct PBalanceSensor_t
{
    AxisSensor_t x;
    AxisSensor_t y;
    AxisSensor_t z;
} ;

struct PBalance_t
{
    PBalanceParam_t param;
    PBalanceSensor_t sensor;
} ;

struct UBalance_t
{
    AxisParam_t param;
    AxisSensor_t sensor;
} ;

#define WIFI_CHAR_LEN 30

#pragma pack(1)
struct WifiParam_t
{
    bool use_wifi;
    char wifi_ssid[WIFI_CHAR_LEN];
    uint8_t wifi_ssid_len;
    char wifi_password[WIFI_CHAR_LEN];
    uint8_t wifi_password_len;
    uint16_t server_port;
};
#pragma pack()

struct DynamicParam_t
{
    bool write_once_default_param;
    bool start_once_p_calibrat;
    bool start_once_u_calibrat;
};

#define FIRST_DOWNLOAD_ADDR 0
#define P_PARAM_ADDR 4
#define U_PARAM_ADDR sizeof(PBalanceParam_t)
#define WIFI_PARAM_ADDR sizeof(PBalanceParam_t) + sizeof(AxisParam_t)

void ParamInit(PBalanceParam_t &_p_param, AxisParam_t & _u_param, WifiParam_t & _wifi_param);
bool SavePBalanceParam(const PBalanceParam_t & _param);
bool SaveUBalanceParam(const AxisParam_t & _param);
bool SaveWiflParam(const WifiParam_t & _param);

void ReadPBalanceParam(PBalanceParam_t & _param);
void ReadUBalanceParam(AxisParam_t & _param);
void ReadWifiParam(WifiParam_t & _param);

bool SaveFirstDownload(uint32_t first);
void ReadFirstDownload(uint32_t & first);