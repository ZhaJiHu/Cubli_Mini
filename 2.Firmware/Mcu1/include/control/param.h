#pragma once
#include "bsp/my_eeprom.h"

struct AxisParam_t
{
    float kp;
    float kv;
    float ks;
    float angle_offset;
} ;

struct AxisSensor_t
{
    float gyro;
    float angle;
    float speed;
} ;

struct PBalanceParam_t
{
    AxisParam_t x;
    AxisParam_t y;
    AxisParam_t z;
    float axis_offset;
} ;

struct PBalanceSensor_t
{
    AxisSensor_t x;
    AxisSensor_t y;
    AxisSensor_t z;
} ;

struct PBalance_t
{
    PBalanceParam_t param;
    PBalanceSensor_t sensor;
} ;

struct UBalance_t
{
    AxisParam_t param;
    AxisSensor_t sensor;
} ;

#define P_PARAM_ADDR 0
#define U_PARAM_ADDR sizeof(PBalanceParam_t)

void ParamInit(PBalanceParam_t &_p_param, AxisParam_t & _u_param);
bool SavePBalanceParam(const PBalanceParam_t & _param);
bool SaveUBalanceParam(const AxisParam_t & _param);
void ReadPBalanceParam(PBalanceParam_t & _param);
void ReadUBalanceParam(AxisParam_t & _param);