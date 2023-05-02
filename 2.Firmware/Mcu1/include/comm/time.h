#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Comm {

class Time
{
   public:
    Time()
    {
        rt_time_           = 0;
        last_time_         = 0;
        update_first_time_ = true;
    }

    float GetTimeUs()
    {
        rt_time_ = micros();
        float ts = (rt_time_ - last_time_) * 1e-6;
        if (ts <= 0)
        {
            ts = 1e-3f;
        }
        last_time_ = rt_time_;
        return ts;
    }

    float GetTimeMs()
    {
        rt_time_ = millis();
        float ts = (rt_time_ - last_time_) * 1e-3;
        if (ts <= 0)
        {
            ts = 1e-3f;
        }
        last_time_ = rt_time_;
        return ts;
    }

    unsigned long GetRtTimeMs() { return millis(); }

    void ResetWaitMs() { update_first_time_ = true; }

    bool WaitMs(float _ms)
    {
        if (update_first_time_)
        {
            first_time_        = millis();
            update_first_time_ = false;
        }
        rt_time_ = millis();
        if (rt_time_ - first_time_ > _ms)
        {
            update_first_time_ = true;
            return true;
        }
        return false;
    }

   private:
    long rt_time_;
    long last_time_;
    bool update_first_time_;
    long first_time_;
};

}  // namespace Comm
}  // namespace CubliMini