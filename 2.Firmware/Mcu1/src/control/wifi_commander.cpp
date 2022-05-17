#include "control/wifi_commander.h"
#include <esp_wifi.h>

namespace CubliMini {
namespace Control {

WifiConnectStatus_e WifiCommander::ConnectWifi()
{
    WiFi.begin(ssid_.c_str(), password_.c_str());
    while (WiFi.status() != WL_CONNECTED) //等待网络连接成功
    {
        delay(500);
        printf("WIFI: connecting...\r\n");
        if(time_out_.WaitMs(1000 * 10)) // 20s连接失败则退出连接
        {
            printf("WIFI: connected fail!\r\n");
            esp_wifi_disconnect();
            esp_wifi_stop();
            wifi_connect_status_ = WifiConnectStatus_e::eWIFI_FAIL;
            return wifi_connect_status_;
        }
    }
    wifi_connect_status_ = WifiConnectStatus_e::eWIFI_SUCCESS;
    printf("WIFI: connected, cubli ip address: %s\r\n", WiFi.localIP().toString()); //打印模块IP

    return wifi_connect_status_;
}


void WifiCommander::LinkMutex(SemaphoreHandle_t * _x_mutex)
{
    x_mutex_ = _x_mutex;
}

void WifiCommander::TcpTask(PBalanceParam_t & _p_parm, AxisParam_t & _u_parm)
{
    if(wifi_connect_status_ == WifiConnectStatus_e::eWIFI_SUCCESS)
    {
        TcpClient(_p_parm, _u_parm);
    }
    else
    {
        printf("WIFI: connected fail!");
    }
}

void WifiCommander::TcpClient(PBalanceParam_t & _p_parm, AxisParam_t & _u_parm)
{
    printf("WIFI: try connect server!!\r\n");
    if (client_.connect(server_ip_, server_port_)) //尝试访问目标地址
    {
        printf("WIFI: successfully connected tcp server! \r\n");
        client_.printf("successfully connected!\r\n"); //向服务器发送数据
        client_.printf("this is Cubli_Mini!\r\n"); //向服务器发送数据
        while (client_.connected() || client_.available()) //如果已连接或有收到的未读取的数据
        {
            if (client_.available()) //如果有数据可读取
            {
                String line = client_.readStringUntil('\n'); //读取数据到换行符
                xSemaphoreTake(*x_mutex_, portMAX_DELAY);
                GetOrSet(line.c_str(), _p_parm, _u_parm);
                xSemaphoreGive(*x_mutex_);
            }

            if(this->output_angle_ && time_out_.WaitMs(100) == true)
            {
                cmd_printf("x_A:%f y_A:%f z_A:%f\r\n", sensor_angle_.x, sensor_angle_.y, sensor_angle_.z);
            }
            delay(20);
        }
        printf("WIFI: close this connect\r\n");
        client_.stop(); //关闭客户端
    }
    else
    {
        printf("WIFI: connect server fail!!\r\n");
        client_.stop(); //关闭客户端
    }
}

void WifiCommander::cmd_printf(const char *format, ...)
{
    char loc_buf[64];
    char * temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);
    if(len < 0) {
        va_end(arg);
        return;
    };
    if(len >= sizeof(loc_buf)){
        temp = (char*) malloc(len+1);
        if(temp == NULL) {
            va_end(arg);
            return;
        }
        len = vsnprintf(temp, len+1, format, arg);
    }
    va_end(arg);
    len = client_.write((uint8_t*)temp, len);
    if(temp != loc_buf){
        free(temp);
    }
}

uint16_t WifiCommander::GetCharLen(const char *user_cmd)
{
    return strlen(user_cmd);
}

} // namespace Cubli 
} // namespace Control 
