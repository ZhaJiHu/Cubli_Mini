#include "control/param.h"

void ParamInit(PBalanceParam_t &_p_param, AxisParam_t & _u_param)
{
    EepromInit();
    ReadPBalanceParam(_p_param);
    ReadUBalanceParam(_u_param);
}

bool SavePBalanceParam(const PBalanceParam_t & _param)
{
    return Write(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void ReadPBalanceParam(PBalanceParam_t & _param)
{
    Read(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool SaveUBalanceParam(const AxisParam_t & _param)
{
    return Write(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void ReadUBalanceParam(AxisParam_t & _param)
{
    Read(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}