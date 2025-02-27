#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "chassis_kinematics.hpp"
#include "robot_config.h"
#include "robot_driver.h"
#include <stdint.h>

#define robot_left_on 30
#define robot_right_on -30
#define robot_left_off -14
#define robot_right_off 11
#define camera_angle_up -22
#define camera_angle_down -76
#define distance_speed 0.6
#define distance_time 0.5
#define camera_acceleration 150

typedef CFF::ChassisVelocities Velocities;

enum CtrlState {
  STARTUP_STATE = 0,
  MANUAL_OPERATION = 1,
  AUTO_OPERATION = 2,
  DEBUG_STATE = 3, // 调试状态
};

/* 可以自行修改增加一些功能 */
enum ManualCtrlState {
  GENERAL_CTRL = 0, // 通用控制
  GENERAL_CTRL2,
  NUM_SUB_STATES // 用于方便地知道子状态的数量，在这个之前添加子状态
};

enum AutoCtrlState {
  WAIT_START = 0,
  START = 1,
  RUNNING = 2,
  FINISH = 3,
};

enum ChassisMode {
  SAFE_MODE = 0,
  SPEED_MODE = 1,
  POSTION_MODE = 2,
};

enum RobotColor {
  RED = 0,
  BLUE,
  YELLOW,
  BLACK,
};

struct position {
  double x;
  double y;
  double z;
  double pitch;
  double roll;
  double yaw;
};

struct pid_data {
  double input;
  double output;
  double setpoint;
};

typedef struct _robot_ {
  uint16_t ctrl_state;
  uint16_t manual_sub_state; // 手动控制的子模式
  uint16_t auto_sub_state;   // 自动控制的子模式

  uint8_t team_color;
  uint8_t auto_path; // 切换不同的自动路线

  float battery_voltage; // 电池电量

  uint16_t buttons; // 手柄按键数据
  uint16_t last_buttons;

  /* 底盘控制使用数据 */
  uint8_t chassis_mode;
  bool imu_hold;               // 底盘是否开启陀螺仪保持
  struct position current_pos; // chassis_mode=POSTION_MODE
  struct position target_pos;
  struct pid_data pid_yaw;
  struct pid_data pid_pos_x;
  struct pid_data pid_pos_y;
  Velocities set_vel; // chassis_mode=SPEED_MODE

  /* 执行机构控制对象 */
  struct switchgear SGs;
  HubMotor *motor[HUB_MOTOR_TOTAL_NUM];
  Servo *servo[SERVO_MOTOR_TOTAL_NUM];
} Robot_t;

extern Robot_t robot;
extern ChassisKinematics *chassis;

#endif /* __ROBOT_H__ */
