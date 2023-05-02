#pragma once
#include <Arduino.h>

#include "control/cubli_mini.h"
#include "control/param.h"

namespace CubliMini {
namespace Control {

#define AUTO_CALIBRATION_P 1
#define AUTO_CALIBRATION_U 2

// GET byte1 byte2 data
#define SCMD_BYTE_1_G 'G'  // get cmd
#define SCMD_BYTE_1_S 'S'  // set cmd
#define SCMD_BYTE_1_C 'C'  // cal
#define SCMD_BYTE_1_F 'F'  // re init
#define SCMD_BYTE_1_R 'R'  // set cmd

#define SCMD_BYTE_2_P 'P'  // point balance param
#define SCMD_BYTE_2_U 'U'  // unilateral balance param
#define SCMD_BYTE_2_S 'S'  // save param
#define SCMD_BYTE_2_W 'W'  // get wifi param

#define SCMD_BYTE_3_X 'X'  //!< x axis
#define SCMD_BYTE_3_Y 'Y'  //!< y axis
#define SCMD_BYTE_3_Z 'Z'  //!< z axis
#define SCMD_BYTE_3_P 'P'  // point balance param
#define SCMD_BYTE_3_U 'U'  // unilateral balance param
#define SCMD_BYTE_3_V 'V'  //!< k2
#define SCMD_BYTE_3_S 'S'  //!< k3
#define SCMD_BYTE_3_A 'A'  //!< angle
#define SCMD_BYTE_3_E 'E'  //!< angle

#define SCMD_BYTE_4_P 'P'  //!< k1
#define SCMD_BYTE_4_V 'V'  //!< k2
#define SCMD_BYTE_4_S 'S'  //!< k3
#define SCMD_BYTE_4_A 'A'  //!< angle

#define MAX_RECEIVER_SIZE 20
const char keol = '\n';

class SerialCommander
{
   public:
    SerialCommander()
    {
        output_angle_ = false;
        rec_cnt_      = 0;
    }

    virtual void cmd_printf(const char *fmt, ...);
    virtual void Run(Stream &_serial, CubliMiniControl &control);

    virtual uint16_t GetCharLen(const char *user_cmd);

    bool output_angle_;

   protected:
    void GetOrSet(const char *user_cmd, CubliMiniControl &control);

    void GetCmd(const char *user_cmd, CubliMiniControl &control);
    void SetCmd(const char *user_cmd, CubliMiniControl &control);

    void SetAxisParam(const char *user_cmd, AxisParam_t &_param);
    void SetPParam(const char *user_cmd, PBalanceParam_t &_param);
    void SetUParam(const char *user_cmd, AxisParam_t &_param);
    void SaveParam(const char *user_cmd, PBalanceParam_t &_p_parm, AxisParam_t &_u_param);

    bool isSentinel(char ch);

    void Calibration(const char *user_cmd, CubliMiniControl &control);

    void SaveWifiParam(const char *user_cmd, CubliMiniControl &control);
    int rec_cnt_;
    char received_chars_[MAX_RECEIVER_SIZE];
};

}  // namespace Control
}  // namespace CubliMini