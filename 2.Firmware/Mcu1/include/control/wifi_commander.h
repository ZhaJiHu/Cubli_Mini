#pragma once
#include "control/serial_commander.h"
#include <WiFi.h>

namespace CubliMini {
namespace Control {

enum WifiConnectStatus_e
{
    eWIFI_FAIL = 0x00,
    eWIFI_SUCCESS = 0x01,
} ;

struct SensorAngle_t
{
    float x;
    float y;
    float z;
};

class WifiCommander : public SerialCommander
{
   public:
    WifiCommander()
    {
        server_ip_.fromString(WIFI_TCP_SERVER_IP);
        server_port_ = WIFI_TCP_SERVER_PORT;
        ssid_ = WIFI_SSID;
        password_ = WIFI_PASSWORD;
        wifi_connect_status_ = eWIFI_FAIL;
    }
    virtual void cmd_printf(const char* fmt, ...);
    virtual uint16_t GetCharLen(const char *user_cmd);

    WifiConnectStatus_e ConnectWifi();
    WifiConnectStatus_e GetConnectStatus()
    {
        return wifi_connect_status_;
    }
    void LinkMutex(SemaphoreHandle_t * _x_mutex);
    void TcpTask(PBalanceParam_t & _p_parm, AxisParam_t & _u_parm);
    void TcpClient(PBalanceParam_t & _p_parm, AxisParam_t & _u_parm);
    void SetServerIp(const IPAddress & _set_ip){ server_ip_ = _set_ip; }
    void SetServerPort(uint16_t _set_port){ server_port_ = _set_port; }
    void SetServerSsid(const String & _set_ssid){ ssid_ = _set_ssid; }
    void SetServerPassword(const String & _set_password){ password_ = _set_password; }

    SensorAngle_t sensor_angle_;
    SemaphoreHandle_t * x_mutex_;

   private:
    WiFiClient client_; //声明一个ESP32客户端对象，用于与服务器进行连接
    IPAddress server_ip_;
    uint16_t server_port_;
    String ssid_;
    String password_;
    Time time_out_;
    WifiConnectStatus_e wifi_connect_status_;
} ;

} // namespace Cubli 
} // namespace Control 
