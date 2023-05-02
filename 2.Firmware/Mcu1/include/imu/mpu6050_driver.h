#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "config/config.h"

using namespace CubliMini::Config;
namespace CubliMini {
namespace ImuDriver {

#define MPU6050_I2CADDR_DEFAULT 0x68  ///< MPU6050 default i2c address w/ AD0 high
#define MPU6050_DEVICE_ID       0x68  ///< The correct MPU6050_WHO_AM_I value

#define MPU6050_SELF_TEST_X       0x0D  ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y       0x0E  ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z       0x0F  ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A       0x10  ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV        0x19  ///< sample rate divisor register
#define MPU6050_CONFIG            0x1A  ///< General configuration register
#define MPU6050_GYRO_CONFIG       0x1B  ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG      0x1C  ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG    0x37  ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE        0x38  ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS        0x3A  ///< Interrupt status register
#define MPU6050_WHO_AM_I          0x75  ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68  ///< Signal path reset register
#define MPU6050_USER_CTRL         0x6A  ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1        0x6B  ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2        0x6C  ///< Secondary power/sleep control register
#define MPU6050_TEMP_H            0x41  ///< Temperature data high byte register
#define MPU6050_TEMP_L            0x42  ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT         0x3B  ///< base address for sensor data reads
#define MPU6050_MOT_THR           0x1F  ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR           0x20  ///< Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

// 行
#define MEAN_FILTER_ROWS 6
// 列
#define MEAN_FILTER_COLS 8

enum GyroConfig_e
{
    eGYRO_250DEGS  = 0x00,
    eGYRO_500DEGS  = 0x08,
    eGYRO_1000DEGS = 0x10,
    eGYRO_2000DEGS = 0x18
};

enum AccConfig_e
{
    eACCEL_2G  = 0x00,
    eACCEL_4G  = 0x08,
    eACCEL_8G  = 0x10,
    eACCEL_16G = 0x18
};

enum DLPFConfig_e
{
    eDLPF_260HZ = 0x00,  // acc delay 0ms gyro 0.98ms
    eDLPF_184HZ = 0x01,  // acc delay 2ms gyro 1.9ms
    eDLPF_94HZ  = 0x02,  // acc delay 3ms gyro 2.8ms
    eDLPF_44HZ  = 0x03   // acc delay 4.9ms gyro 4.8ms
};

struct EulerAngle_t
{
    float roll;
    float pitch;
    float yaw;
};

struct Gyro_t
{
    float gx;
    float gy;
    float gz;
};

struct Acc_t
{
    float ax;
    float ay;
    float az;
};

struct ImuData_t
{
    float temp;
    Acc_t acc;
    Gyro_t gyro;
    EulerAngle_t angle;
};

struct RawGyro_t
{
    volatile int16_t gx;
    volatile int16_t gy;
    volatile int16_t gz;
};

struct RawAcc_t
{
    volatile int16_t ax;
    volatile int16_t ay;
    volatile int16_t az;
};

struct ImuRawData_t
{
    volatile int16_t temp;
    RawGyro_t gyro;
    RawAcc_t acc;
};

class Mpu6050Driver
{
   public:
    Mpu6050Driver()
    {
        memset(&gyro_offset_, 0, sizeof(gyro_offset_));
        memset(&mean_imu_raw_data_, 0, sizeof(mean_imu_raw_data_));
        memset(&mean_filter_fifo, 0, sizeof(mean_filter_fifo));
        imu_init_seccessfully_ = false;
        interrupt_flag_        = false;
        acc_config_            = eACCEL_4G;
        gyro_config_           = eGYRO_1000DEGS;
    }

    bool Init(
        TwoWire *_wire,
        int _sda_pin       = IMU_SDA_PIN,
        int _scl_pin       = IMU_SCL_PIN,
        int _interrupt_pin = IMU_INTERRUPT_PIN);
    void ImuGetRtValue(ImuData_t &_imu_data, ImuRawData_t &_imu_raw_data);
    bool GetMPU6050InterruptFlag();
    void ResetMPU6050InterruptFlag();

   private:
    bool MPU6050Init();
    uint8_t GetMPU6050Id();
    void SetAccRange(const AccConfig_e &_acc_config);
    void SetGyroRange(const GyroConfig_e &_gyro_config);

    uint8_t I2cWrite(int _address, int _res, int _data);
    uint8_t I2cRead(int _address, int _res, uint8_t *_buff, int _size);

   private:
    void CalGyroOffset();
    void MPU6050ReadData(ImuRawData_t &_imu_raw_data, ImuData_t &_imu_data);
    void MeanFilter(ImuRawData_t &_imu_raw_data);
    void MPU6050InterruptInit();

   public:
    TwoWire *i2c_;
    int sda_pin_;
    int scl_pin_;
    int interrupt_pin_;
    int gyro_range_;
    int acc_range_;
    bool imu_init_seccessfully_;
    bool interrupt_flag_;

    AccConfig_e acc_config_;
    GyroConfig_e gyro_config_;

   private:
    ImuRawData_t mean_imu_raw_data_;
    Gyro_t gyro_offset_;

   private:
    int32_t mean_filter_fifo[6][15];
};

}  // namespace ImuDriver
}  // namespace CubliMini
