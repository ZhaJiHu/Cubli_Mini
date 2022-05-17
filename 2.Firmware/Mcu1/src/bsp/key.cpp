#include "bsp/key.h"

namespace CubliMini {
namespace Bsp {   


void KeyDriver::Init(int _key_pin)
{
    key_pin_ = _key_pin;
    pinMode(key_pin_, INPUT_PULLUP);
}

KeyAction_e KeyDriver::GetKeyAction()
{   
    KeyAction_e key_action = eKEY_ACTION_NO_PRESS;
    bool get_key = digitalRead(key_pin_);
    if(get_key == KEY_DOWM && key_press_status_ == eKEY_STATUS_NOMAL)
    {
        key_is_press_start_time_ = key_time_.GetRtTimeMs();
        key_press_status_ = eKEY_STATUS_DOWM;
    }
    switch(key_press_status_)
    {
        case eKEY_STATUS_DOWM:
            {
                if(get_key == KEY_UP)
                {
                    //消抖
                    if((key_time_.GetRtTimeMs() - key_is_press_start_time_) > 50)
                    {
                        key_press_status_ = eKEY_STATUS_UP;
                    }
                }
                else
                {   
                    // 超过2S判断为长按
                    if((key_time_.GetRtTimeMs() - key_is_press_start_time_) > 2000)
                    {
                        key_press_status_ = eKEY_STATUS_IS_STILL_DOWM;
                        key_action = eKEY_ACTION_LONG_PRESS;
                        return key_action;
                    }
                }
            }
            break;
        case eKEY_STATUS_UP:
            {
                key_action = eKEY_ACTION_SHORT_PRESS;
                key_press_status_ = eKEY_STATUS_NOMAL;
            }
            break;
        case eKEY_STATUS_NOMAL:
            {
                key_action = eKEY_ACTION_NO_PRESS;
            }
            break;
        case eKEY_STATUS_IS_STILL_DOWM:
            {
                key_action = eKEY_ACTION_NO_PRESS;
                if(get_key == KEY_UP)
                {
                    key_press_status_ = eKEY_STATUS_NOMAL;
                }
            }
            break;
    }
    return key_action;
}

} // namespace Cubli 
} // namespace Bsp 