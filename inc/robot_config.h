#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#define ROBOT_VERSIONS                  "v1.1"

#define ROBOT_CHASSIS_OMNI              0    //4轮全向轮底盘
#define ROBOT_CHASSIS_THREE_WHEEL_OMNI  1    //3轮全向轮底盘
#define ROBOT_CHASSIS_FOUR_DIFF         2    //4轮差速底盘
#define ROBOT_CHASSIS_TWO_DIFF          3    //2轮差速底盘

#define ROBOT_CHASSIS                   ROBOT_CHASSIS_TWO_DIFF  //修改机器人底盘类型

#define SWITCHGEAR_4_USE_PWM            //外设接口4启用PWM功能，PWM可以用来控制无刷电调
#define SWITCHGEAR_6_CATAPULT           //外设接口6启用弹射蓄能机构

#define WHEELS_DIAMETER            0.08 //轮子直径 unit: m

#define OMNI4_WHEELS_TRACK         0.133 //4轮全向底盘轮距 unit: m
#define OMNI4_WHEELS_BASE          0.133 //4轮全向底盘轴距 unit: m
#define OMNI3_CENTER_DISTANCE      0.094 //3轮全向底盘轮中心距（半径） unit: m
#define DIFF_WHEELS_TRACK          0.157 //差速底盘轮距 unit: m
#define DIFF_WHEELS_BASE           0.157 //差速底盘轴距 unit: m

/* 代码段运行频率设置 */
#define DO_CHASSIS_CTRL_FREQUENCY  100  // hz
#define PID_CALC_FREQUENCY         50   // hz
#define IMU_UPDATE_FREQUENCY       100  // hz

#define HUB_MOTOR_TOTAL_NUM        4    // 轮毂电机总个数
#define SERVO_MOTOR_TOTAL_NUM      3    // 舵机总个数

#define HUB_MOTOR_MAX_SPEED        200  // 轮毂电机最大转速 unit: RPM
#define CHASSIS_MAX_ACC            0.1  // 底盘最大加速度 m/s
#define CHASSIS_MAX_LINEAR_SPEED   0.6  // 底盘最大速度 m/s
#define CHASSIS_MAX_ANGULAR_SPEED  PI*0.75   // 底盘最大速度 rad/s

#define VISUAL_COMM_USE_UART       "uart3" //视觉通讯使用的串口

#define LOW_BATTERY_ALARM_ENABLED       // 低电报警

/*
    ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
#define LEFT_FRONT_MOTOR_ID     1  /* 根据底盘电机ID修改 */
#define RIGHT_FRONT_MOTOR_ID    2
#define LEFT_BACK_MOTOR_ID      3
#define RIGHT_BACK_MOTOR_ID     4

#endif /* __ROBOT_CONFIG_H__ */