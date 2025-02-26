#ifndef __ROBOT_DRIVER_H__
#define __ROBOT_DRIVER_H__
#include <rtthread.h>
#include "easy_math.h"
#include "cff_utils.h"
#include "cff_types.h"
#include "pins_port.h"
#include "easyflash.h"

#include "bus_servo.hpp"
#include "hub_motor.hpp"
#include "TinyFrame.h"

#include "AHRS.h"

#include "beep_port.h"
#include "tone_player.hpp"
#include "music_code.h"

#include "tusbh.h"
#include "tusbh_hid.h"
#include "hid.h"
#include "pid.h"

using namespace CFF;

#define TIME2_PWM_PERIOD  20000000 // 500HZ

#define SG4_PWM_DEV_CHANNEL     2
#define SG4_PWM_START_VALUE     19000000
#define SG4_PWM_MAX_VALUE       18000000

#define TEMP_PWM_DEV_CHANNEL    1
#define TEMP_PID_MAX_OUT        2000000.0f
#define TEMP_PID_KP             100000.0f
#define TEMP_PID_KI             10000.0f
#define TEMP_PID_KD             0.0f
#define IMU_MAX_TEMP            50.0f
#define GYRO_OFFSET_KP          0.001f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大

#define GAMEPAD_DEAD_ZONE 5

typedef hid_gamepad_xbox_report_t Gamepad_t;

/**
 * @brief setLedColor(color) 可用的LED颜色选项
 * 
 */
enum LED_Color
{
    LED_COLOR_OFF     = 0b000, // 关闭状态，无颜色
    LED_COLOR_RED     = 0b001, // 只有红色
    LED_COLOR_GREEN   = 0b010, // 只有绿色
    LED_COLOR_YELLOW  = 0b011, // 红色+绿色=黄色
    LED_COLOR_BLUE    = 0b100, // 只有蓝色
    LED_COLOR_MAGENTA = 0b101, // 红色+蓝色=品红色
    LED_COLOR_CYAN    = 0b110, // 绿色+蓝色=青色
    LED_COLOR_WHITE   = 0b111  // 红色+绿色+蓝色=白色
};

typedef struct switchgear
{
    uint16_t SG1_PWR : 1;
    uint16_t SG1_IO : 1;
    uint16_t SG2_PWR : 1;
    uint16_t SG2_IO : 1;
    uint16_t SG3_PWR : 1;
    uint16_t SG3_IO : 1;
    uint16_t SG4_PWR : 1;
    uint16_t SG4_IO : 1;
    uint16_t SG5_PWR : 1;
    uint16_t SG5_IO : 1;
    uint16_t SG6_PWR : 1;
    uint16_t SG6_IO : 1;
    uint16_t reserve : 4;
} switchgear_t;

typedef struct
{
    int16_t type;
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    uint32_t timestamp;
} visual_t;

namespace io
{
    void setLedColor(uint8_t color);
    void setSG1(uint8_t pwr_state, uint8_t io_state);
    void setSG2(uint8_t pwr_state, uint8_t io_state);
    void setSG3(uint8_t pwr_state, uint8_t io_state);
    void setSG4(uint8_t pwr_state, uint8_t io_state);
    void setSG5(uint8_t pwr_state, uint8_t io_state);
    void setSG6(uint8_t pwr_state, uint8_t io_state);
    int getIN1();
    int getIN2();
    int getIN3();
    int getIN4();
}

namespace servo
{
    ServoPort *PortPointer(void);
    Servo *Register(uint8_t id, uint8_t type = ServoType::SCS, float max_angle = 180, float min_angle = -180);
}

namespace motor
{
    HubMotorPort *PortPointer(void);
    HubMotor *Register(uint8_t id);
}

namespace imu
{
    void getQuaternion(Quaternion *q);
    void getAngle(double *roll, double *pitch, double *yaw);
    void getGyro(double *gx, double *gy, double *gz);
    void getAccle(double *ax, double *ay, double *az);
    struct sensor_3_axis *getGyro(void);
    void reset(void);
    bool isCalibration(void);
    void startCalibration(int argc, char **argv);
}

namespace audio
{
    bool playMusic(const char *name, bool loop=false);
    void stop(void);
}

namespace visual
{
    visual_t *getBall(uint8_t num);
    void startDebug(void);
    void stopDebug(void);
}

namespace usb
{
    Gamepad_t *GamepadPointer(void);
    bool isConnected(void);
}


#endif /* __ROBOT_DRIVER_H__ */