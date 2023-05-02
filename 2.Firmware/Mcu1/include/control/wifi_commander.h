#pragma once
#include <WiFi.h>

#include "control/serial_commander.h"

namespace CubliMini {
namespace Control {

enum WifiConnectStatus_e
{
    eWIFI_FAIL    = 0x00,
    eWIFI_SUCCESS = 0x01,
};

struct SensorAngle_t
{
    float x;
    float y;
    float z;
};

class WifiCommander : public SerialCommander
{
   public:
    WifiCommander() : wifi_connect_status_(eWIFI_FAIL)
    {
        server_port_ = WIFI_DEFAULT_TCP_SERVER_PORT;
        ssid_        = WIFI_DEFAULT_SSID;
        password_    = WIFI_DEFAULT_PASSWORD;
    }

    virtual void cmd_printf(const char *fmt, ...);
    virtual uint16_t GetCharLen(const char *user_cmd);

    WifiConnectStatus_e ConnectWifi();
    WifiConnectStatus_e GetConnectStatus() { return wifi_connect_status_; }

    void SetOption(const WifiParam_t &param);
    void WaitClientConnect();

    void TcpClientUnpack(CubliMiniControl &control);

    SensorAngle_t sensor_angle_;

   private:
    WiFiServer server_;
    WiFiClient client_;  // 声明一个ESP32客户端对象，用于与服务器进行连接
    WifiParam_t wifi_param_;

    uint16_t server_port_;
    String ssid_;
    String password_;
    WifiConnectStatus_e wifi_connect_status_;
};

}  // namespace Control
}  // namespace CubliMini
