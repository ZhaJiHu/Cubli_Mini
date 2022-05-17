#include "imu/ahrs.h"

namespace CubliMini {
namespace Imu {  

void AHRS::ImuAHRSUpdate(Q_t &_q,  const ImuData_t &_imu_data)
{
  static volatile float ex_dt, ey_dt, ez_dt;
  volatile float gx, gy, gz, ax, ay, az;
  float norm;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez, halfT;
  float tempq0, tempq1, tempq2, tempq3;

  float q0q0 = _q.q0 * _q.q0;
  float q0q1 = _q.q0 * _q.q1;
  float q0q2 = _q.q0 * _q.q2;
  float q0q3 = _q.q0 * _q.q3;
  float q1q1 = _q.q1 * _q.q1;
  float q1q2 = _q.q1 * _q.q2;
  float q1q3 = _q.q1 * _q.q3;
  float q2q2 = _q.q2 * _q.q2;
  float q2q3 = _q.q2 * _q.q3;
  float q3q3 = _q.q3 * _q.q3;

  gx = _imu_data.gyro.gx * (M_PI / 180.0f);
  gy = _imu_data.gyro.gy * (M_PI / 180.0f);
  gz = _imu_data.gyro.gz * (M_PI / 180.0f);
  ax = _imu_data.acc.ax;
  ay = _imu_data.acc.ay;
  az = _imu_data.acc.az;

  halfT = time_.GetTimeUs() / 2;

  norm = invSqrt(ax * ax + ay * ay + az * az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加计的三维向量转成单位向量。

  // estimated direction of gravity and flux (v and w)
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
    ex_dt = ex_dt + ex * ki_ * halfT;
    ey_dt = ey_dt + ey * ki_ * halfT;
    ez_dt = ez_dt + ez * ki_ * halfT;
    // 用叉积误差来做PI修正陀螺零偏
    gx = gx + kp_ * ex + ex_dt;
    gy = gy + kp_ * ey + ey_dt;
    gz = gz + kp_ * ez + ez_dt;
  } 
  // 四元数微分方程
  tempq0 = _q.q0 + (-_q.q1 * gx - _q.q2 * gy - _q.q3 * gz) * halfT;
  tempq1 = _q.q1 + (_q.q0 * gx + _q.q2 * gz - _q.q3 * gy) * halfT;
  tempq2 = _q.q2 + (_q.q0 * gy - _q.q1 * gz + _q.q3 * gx) * halfT;
  tempq3 = _q.q3 + (_q.q0 * gz + _q.q1 * gy - _q.q2 * gx) * halfT;

  // 四元数规范化
  norm = invSqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
  _q.q0 = tempq0 * norm;
  _q.q1 = tempq1 * norm;
  _q.q2 = tempq2 * norm;
  _q.q3 = tempq3 * norm;
}

void AHRS::ConvertToEulerAmgleByQ(EulerAngle_t &_angle, const Q_t &_q)
{
   _angle.yaw = -atan2(2 * _q.q0 * _q.q2 + 2 *_q.q0 * _q.q3, -2 * _q.q2 * _q.q2 - 2 * _q.q3 * _q.q3 + 1) * 180 / M_PI; // yaw   -pi----pi
   _angle.pitch = -asin(-2 * _q.q1 * _q.q3 + 2 *_q.q0 * _q.q2) * 180 / M_PI;                                           // pitch -pi/2    --- pi/2
   _angle.roll = atan2(2 * _q.q2 * _q.q3 + 2 *_q.q0 * _q.q1, -2 * _q.q1 * _q.q1 - 2 * _q.q2 * _q.q2 + 1) * 180 / M_PI; // roll  -pi-----pi
}

void AHRS::ImuIsStatic()
{
    static uint32_t static_frame_num = 0;
    if(fabs(imu_data_.gyro.gx) < IMU_STATIC_GYRO_THRESHOLD &&
       fabs(imu_data_.gyro.gy) < IMU_STATIC_GYRO_THRESHOLD &&
       fabs(imu_data_.gyro.gz) < IMU_STATIC_GYRO_THRESHOLD)
    {
        if(is_static_ == false)
        {
            static_frame_num ++;
            if(static_frame_num > 500)
            {
                static_frame_num = 0;
                is_static_ = true;
            }
        }
    }
    else
    {
        is_static_ = false;
        static_frame_num = 0;
    }
}

bool AHRS::ImuUpdate()
{
  if(mpu6050_driver.GetMPU6050InterruptFlag())
  {
      mpu6050_driver.ResetMPU6050InterruptFlag();
      mpu6050_driver.ImuGetRtValue(imu_data_, imu_raw_data_);
      ImuIsStatic();
      ImuAHRSUpdate(q_, imu_data_);
      ConvertToEulerAmgleByQ(imu_data_.angle, q_);
      return true;
  }
  return false;
}

float AHRS::invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

} // namespace Cubli 
} // namespace Imu 