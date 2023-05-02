#include "imu/mpu6050_driver.h"

namespace CubliMini {
namespace ImuDriver {

bool interrupt_flag = false;

void ARDUINO_ISR_ATTR InterruptFuntion() { interrupt_flag = true; }

bool Mpu6050Driver::GetMPU6050InterruptFlag()
{
    interrupt_flag_ = interrupt_flag;
    return interrupt_flag_;
}

void Mpu6050Driver::ResetMPU6050InterruptFlag() { interrupt_flag = interrupt_flag_ = false; }

void Mpu6050Driver::MPU6050InterruptInit()
{
    pinMode(interrupt_pin_, INPUT_PULLUP);
    attachInterrupt(interrupt_pin_, InterruptFuntion, FALLING);
}

bool Mpu6050Driver::Init(TwoWire *_wire, int _sda_pin, int _scl_pin, int _interrupt_pin)
{
    i2c_ = _wire;
    i2c_->begin(_sda_pin, _scl_pin, (uint32_t)400000);
    interrupt_pin_ = _interrupt_pin;
    SetAccRange(AccConfig_e::eACCEL_4G);
    SetGyroRange(GyroConfig_e::eGYRO_1000DEGS);
    if (MPU6050Init() == true)
    {
        printf("IMU: init successful!!!\r\n");
    }
    else
    {
        printf("IMU: init fail!!!\r\n");
    }
    MPU6050InterruptInit();
    delay(100);
    CalGyroOffset();
    return true;
}

void Mpu6050Driver::ImuGetRtValue(ImuData_t &_imu_data, ImuRawData_t &_imu_raw_data)
{
    MPU6050ReadData(_imu_raw_data, _imu_data);
}

void Mpu6050Driver::SetAccRange(const AccConfig_e &_acc_config)
{
    acc_config_ = _acc_config;
    switch (acc_config_)
    {
    case eACCEL_2G:
        acc_range_ = 2;
        break;
    case eACCEL_4G:
        acc_range_ = 4;
        break;
    case eACCEL_8G:
        acc_range_ = 8;
        break;
    case eACCEL_16G:
        acc_range_ = 16;
        break;
    default:
        acc_range_  = 2;
        acc_config_ = eACCEL_2G;
        break;
    }
}

void Mpu6050Driver::SetGyroRange(const GyroConfig_e &_gyro_config)
{
    gyro_config_ = _gyro_config;
    switch (gyro_config_)
    {
    case eGYRO_250DEGS:
        gyro_range_ = 250;
        break;
    case eGYRO_500DEGS:
        gyro_range_ = 500;
        break;
    case eGYRO_1000DEGS:
        gyro_range_ = 1000;
        break;
    case eGYRO_2000DEGS:
        gyro_range_ = 2000;
        break;
    default:
        gyro_range_  = 1000;
        gyro_config_ = eGYRO_1000DEGS;
        break;
    }
}

bool Mpu6050Driver::MPU6050Init()
{
    if (uint8_t _id = GetMPU6050Id() != MPU6050_DEVICE_ID)
    {
        printf("IMU: can not find mpu6050, get id: \r\n", _id);
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_PWR_MGMT_1, 0x01) != ESP_OK)  // 解除休眠模式
    {
        printf("IMU: 0x01 \r\n");
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_CONFIG, DLPFConfig_e::eDLPF_94HZ) != ESP_OK)
    {
        printf("IMU: 0x02 \r\n");
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_GYRO_CONFIG, gyro_config_) != ESP_OK)
    {
        printf("IMU: 0x03 \r\n");
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_ACCEL_CONFIG, acc_config_) != ESP_OK)
    {
        printf("IMU: 0x04 \r\n");
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_INT_PIN_CONFIG, 0x02) !=
        ESP_OK)  // logic level for the INT pin is active high
    {
        printf("IMU: 0x05 \r\n");
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_INT_ENABLE, 0x01) !=
        ESP_OK)  // able data ready interrupt
    {
        printf("IMU: 0x06 \r\n");
        return false;
    }

    if (I2cWrite(MPU6050_I2CADDR_DEFAULT, MPU6050_SMPLRT_DIV, 0x00) !=
        ESP_OK)  // Sample Rate: Gyro output rate / (1 + 0) = 1000Hz
    {
        printf("IMU: 0x07 \r\n");
        return false;
    }

    delay(100);

    imu_init_seccessfully_ = true;
    return true;
}

uint8_t Mpu6050Driver::GetMPU6050Id()
{
    uint8_t _id;
    I2cRead(MPU6050_I2CADDR_DEFAULT, MPU6050_WHO_AM_I, &_id, 1);
    return _id;
}

uint8_t Mpu6050Driver::I2cWrite(int _address, int _res, int _data)
{
    i2c_->beginTransmission(_address);
    i2c_->write(_res);
    i2c_->write(_data);
    return i2c_->endTransmission();
}

uint8_t Mpu6050Driver::I2cRead(int _address, int _res, uint8_t *_buff, int _size)
{
    i2c_->beginTransmission(_address);
    i2c_->write(_res);
    i2c_->endTransmission();
    // read data
    uint8_t read_len = i2c_->requestFrom(_address, _size);
    for (byte i = 0; i < read_len; ++i)
    {
        _buff[i] = i2c_->read();
    }
    return read_len;
}

void Mpu6050Driver::MPU6050ReadData(ImuRawData_t &_imu_raw_data, ImuData_t &_imu_data)
{
    uint8_t mpu_buf[14] = {0};

    I2cRead(MPU6050_I2CADDR_DEFAULT, MPU6050_ACCEL_OUT, mpu_buf, 14);
    _imu_raw_data.acc.ax = (((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];  // ax
    _imu_raw_data.acc.ay = (((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];  // ay
    _imu_raw_data.acc.az = (((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];  // az

    _imu_raw_data.temp = (((int16_t)mpu_buf[6]) << 8) | mpu_buf[7];  // temp

    _imu_raw_data.gyro.gx = (((int16_t)mpu_buf[8]) << 8) | mpu_buf[9];    // gx
    _imu_raw_data.gyro.gy = (((int16_t)mpu_buf[10]) << 8) | mpu_buf[11];  // gy
    _imu_raw_data.gyro.gz = (((int16_t)mpu_buf[12]) << 8) | mpu_buf[13];  // gz

    MeanFilter(_imu_raw_data);

    _imu_raw_data = mean_imu_raw_data_;

    // 转为g
    _imu_data.acc.ax = (float)_imu_raw_data.acc.ax / (32768 / acc_range_);
    _imu_data.acc.ay = (float)_imu_raw_data.acc.ay / (32768 / acc_range_);
    _imu_data.acc.az = (float)_imu_raw_data.acc.az / (32768 / acc_range_);

    // 转成度每秒
    _imu_data.gyro.gx = (float)_imu_raw_data.gyro.gx / (32768 / gyro_range_) - gyro_offset_.gx;
    _imu_data.gyro.gy = (float)_imu_raw_data.gyro.gy / (32768 / gyro_range_) - gyro_offset_.gy;
    _imu_data.gyro.gz = (float)_imu_raw_data.gyro.gz / (32768 / gyro_range_) - gyro_offset_.gz;
    _imu_data.temp    = 36.53f + (float)_imu_raw_data.temp / 340;
}

//[0]-[9]为最近10次数据 [10]为10次数据的平均值
void Mpu6050Driver::MeanFilter(ImuRawData_t &_imu_raw_data)
{
    for (int rows = 0; rows < MEAN_FILTER_ROWS; ++rows)
    {
        for (int cols = 1; cols < MEAN_FILTER_COLS; ++cols)
        {
            mean_filter_fifo[rows][cols - 1] = mean_filter_fifo[rows][cols];
        }
    }

    mean_filter_fifo[0][MEAN_FILTER_COLS - 1] = _imu_raw_data.acc.ax;
    mean_filter_fifo[1][MEAN_FILTER_COLS - 1] = _imu_raw_data.acc.ay;
    mean_filter_fifo[2][MEAN_FILTER_COLS - 1] = _imu_raw_data.acc.az;
    mean_filter_fifo[3][MEAN_FILTER_COLS - 1] = _imu_raw_data.gyro.gx;
    mean_filter_fifo[4][MEAN_FILTER_COLS - 1] = _imu_raw_data.gyro.gy;
    mean_filter_fifo[5][MEAN_FILTER_COLS - 1] = _imu_raw_data.gyro.gz;

    for (int rows = 0; rows < MEAN_FILTER_ROWS; ++rows)
    {
        int32_t sum = 0;
        for (int cols = 0; cols < MEAN_FILTER_COLS; ++cols)
        {
            sum += mean_filter_fifo[rows][cols];
        }
        mean_filter_fifo[rows][MEAN_FILTER_COLS] = sum / MEAN_FILTER_COLS;
    }

    mean_imu_raw_data_.acc.ax  = mean_filter_fifo[0][MEAN_FILTER_COLS];
    mean_imu_raw_data_.acc.ay  = mean_filter_fifo[1][MEAN_FILTER_COLS];
    mean_imu_raw_data_.acc.az  = mean_filter_fifo[2][MEAN_FILTER_COLS];
    mean_imu_raw_data_.gyro.gx = mean_filter_fifo[3][MEAN_FILTER_COLS];
    mean_imu_raw_data_.gyro.gy = mean_filter_fifo[4][MEAN_FILTER_COLS];
    mean_imu_raw_data_.gyro.gz = mean_filter_fifo[5][MEAN_FILTER_COLS];
}

void Mpu6050Driver::CalGyroOffset()
{
    bool cal_gyro_offset_seccess = false;
    ImuData_t offset_data;
    ImuRawData_t offset_raw_data;
    Gyro_t gyro_sum;
    uint32_t count    = 0;
    uint32_t cal_time = 0;
    if (imu_init_seccessfully_)
    {
        while (cal_gyro_offset_seccess == false)
        {
            MPU6050ReadData(offset_raw_data, offset_data);
            if (fabs(offset_data.gyro.gx) < IMU_INIT_GYRO_THRESHOLD &&
                fabs(offset_data.gyro.gy) < IMU_INIT_GYRO_THRESHOLD &&
                fabs(offset_data.gyro.gz) < IMU_INIT_GYRO_THRESHOLD)
            {
                count++;
                gyro_sum.gx += offset_data.gyro.gx;
                gyro_sum.gy += offset_data.gyro.gy;
                gyro_sum.gz += offset_data.gyro.gz;
                if (count > 1500)
                {
                    cal_gyro_offset_seccess = true;
                    gyro_offset_.gx         = gyro_sum.gx / 1500;
                    gyro_offset_.gy         = gyro_sum.gy / 1500;
                    gyro_offset_.gz         = gyro_sum.gz / 1500;
                    printf(
                        "IMU: calibrate offset seccess, gx: %f gy: %f gz: %f\r\n",
                        gyro_offset_.gx,
                        gyro_offset_.gy,
                        gyro_offset_.gz);
                }
            }
            else
            {
                if (cal_time % 200 == 0)
                {
                    printf("IMU: calibrate offset fail, please keep imu static!\r\n");
                    printf(
                        "rt_gx: %f gy: %f gz: %f\r\n",
                        offset_data.gyro.gx,
                        offset_data.gyro.gy,
                        offset_data.gyro.gz);
                }
                count       = 0;
                gyro_sum.gx = 0;
                gyro_sum.gy = 0;
                gyro_sum.gz = 0;
            }

            if (cal_time > 10 * 1000)  // 10s
            {
                printf("IMU: caloffset fail, timeout!\r\n");
                gyro_offset_.gx = 0;
                gyro_offset_.gy = 0;
                gyro_offset_.gz = 0;
                break;
            }
            cal_time++;
            delay(1);
        }
    }
    else
    {
        printf("IMU: is not init, offset fail !!!\r\n");
    }
}

}  // namespace ImuDriver
}  // namespace CubliMini
