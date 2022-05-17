#include "bsp/can.h"

CAN_device_t CAN_cfg;

namespace Cubli
{
namespace Bsp
{

void CanDriver::Init(int _can_txd_pin, int _can_rxd_pin, CAN_speed_t _can_speed)
{
    CAN_cfg.tx_pin_id = (gpio_num_t)_can_txd_pin;
    CAN_cfg.rx_pin_id = (gpio_num_t)_can_rxd_pin;
    CAN_cfg.speed = _can_speed;
    CAN_cfg.rx_queue = xQueueCreate(100, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
}

void CanDriver::CanSendMotorSpeed(float _send_ch2_speed, float _send_ch3_speed)
{
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = eCAN_SEND_MOTOR_SPEED_FRAME;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = (int32_t)(_send_ch2_speed * 1000.0f) & 0xff;
    tx_frame.data.u8[1] = (int32_t)(_send_ch2_speed * 1000.0f) >> 8;
    tx_frame.data.u8[2] = (int32_t)(_send_ch2_speed * 1000.0f) >> 16;
    tx_frame.data.u8[3] = (int32_t)(_send_ch2_speed * 1000.0f) >> 24;
    tx_frame.data.u8[4] = (int32_t)(_send_ch3_speed * 1000.0f) & 0xff;
    tx_frame.data.u8[5] = (int32_t)(_send_ch3_speed * 1000.0f) >> 8;
    tx_frame.data.u8[6] = (int32_t)(_send_ch3_speed * 1000.0f) >> 16;
    tx_frame.data.u8[7] = (int32_t)(_send_ch3_speed * 1000.0f) >> 24;
    ESP32Can.CANWriteFrame(&tx_frame);
}

void CanDriver::CanGetMotorSpeed(float &_set_ch2_speed, float &_set_ch3_speed)
{
    CAN_frame_t rx_frame;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE)
    {
        // 标准帧和数据帧
        if (rx_frame.FIR.B.FF == CAN_frame_std && rx_frame.FIR.B.RTR == CAN_no_RTR)
        {
            if (rx_frame.MsgID == eCAN_GET_MOTOR_SPEED_FRAME)
            {
                _set_ch2_speed = (float)(rx_frame.data.u8[0] | rx_frame.data.u8[1] << 8 | rx_frame.data.u8[2] << 16 | rx_frame.data.u8[3] << 24) / 1000.0f;
                _set_ch3_speed = (float)(rx_frame.data.u8[4] | rx_frame.data.u8[5] << 8 | rx_frame.data.u8[6] << 16 | rx_frame.data.u8[7] << 24) / 1000.0f;
                rx_frame_count_ = 0;
                can_is_online_ = eON_LINE;
            }
        }
    }
}

CanStatus_e CanDriver::CanIsOnline()
{
    rx_frame_count_++;
    if (rx_frame_count_ > CAN_OFFLINE_COUNT)
    {
        can_is_online_ = eOFF_LINE;
    }
    return can_is_online_;
}

} // namespace Cubli
} // namespace Bsp