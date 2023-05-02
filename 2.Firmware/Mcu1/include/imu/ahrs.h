#pragma once

#include <Arduino.h>

#include "comm/time.h"
#include "imu/mpu6050_driver.h"

using namespace CubliMini::ImuDriver;
namespace CubliMini {
namespace Imu {

#define Kp 5.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f  // integral gain governs rate of convergence of gyroscope biases

struct Q_t
{
    volatile float q0;
    volatile float q1;
    volatile float q2;
    volatile float q3;
};

class AHRS
{
   public:
    AHRS()
    {
        q_.q0 = 1.0f;
        q_.q1 = 0;
        q_.q2 = 0;
        q_.q3 = 0;
        memset(&imu_data_, 0, sizeof(imu_data_));
        memset(&imu_raw_data_, 0, sizeof(imu_raw_data_));
        kp_ = 1.0f;
        ki_ = 0.01f;
    };
    bool ImuUpdate();
    void ImuIsStatic();

   public:
    ImuData_t imu_data_;
    ImuRawData_t imu_raw_data_;
    Mpu6050Driver mpu6050_driver;
    Q_t q_;
    Comm::Time time_;
    float kp_;
    float ki_;
    bool is_static_;

   private:
    float invSqrt(float x);
    float GetTimeUs();
    void ImuAHRSUpdate(Q_t &_q, const ImuData_t &_imu_data);
    void ConvertToEulerAmgleByQ(EulerAngle_t &_angle, const Q_t &_q);
};

}  // namespace Imu
}  // namespace CubliMini