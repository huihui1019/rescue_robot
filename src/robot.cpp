#include "robot.h"
#include "robot_function.h"

using namespace CFF;

Robot_t robot;                   /**< 机器人对象 */
ChassisKinematics *chassis;      /**< 底盘运动学对象 */
Gamepad_t *gamepad;              /**< 手柄对象 */
HubMotorPort *motor_port;        /**< 电机端口对象 */
ServoPort *servo_port;           /**< 舵机端口对象 */
static uint32_t tTime[10] = {0}; /**< 时间戳数组 */

/* 用这三个PID在自动阶段控制小车的位置 */
PID position_x_pid; /**< X轴位置PID控制器 */
PID position_y_pid; /**< Y轴位置PID控制器 */
PID yaw_pid;        /**< 偏航角PID控制器 */

/* 声明函数 */
void auto_mode_path(void *parameter);
void motor_reset();
namespace manual_ctrl {
/**
 * @brief 使用手柄的菜单键切换子模式
 *
 * @param[in] robot        指向机器人的指针，用于获取和设置机器人的状态。
 * @param[in] button_clicked 指示菜单键是否被点击的布尔值。
 */
void SubCtrlState(Robot_t *robot, bool button_clicked);

/**
 * @brief 通过手柄按钮控制不同外设输出
 *
 * @param[in] robot     指向机器人的指针，用于控制外设。
 * @param[in] gamepad   指向手柄状态的指针，用于获取按钮点击事件。
 */
void General(Robot_t *robot, Gamepad_t *gamepad);

/**
 * @brief 通过手柄按钮控制不同外设输出（第二种模式）
 *
 * @param[in] robot     指向机器人的指针，用于控制外设。
 * @param[in] gamepad   指向手柄状态的指针，用于获取按钮点击事件。
 */
void General_2(Robot_t *robot, Gamepad_t *gamepad);

/**
 * @brief 手柄切换自动模式路径
 *
 * @param[in] robot     指向机器人的指针，用于设置自动模式路径和状态。
 * @param[in] gamepad   指向手柄状态的指针，用于获取按钮点击事件。
 */
void AutoSwitch(Robot_t *robot, Gamepad_t *gamepad);

/**
 * @brief 手柄底盘速度控制
 *
 * @param[in] robot     指向机器人的指针，用于设置底盘速度和目标位置。
 * @param[in] gamepad   指向手柄状态的指针，用于获取遥感输入。
 */
void ChassisSpeed(Robot_t *robot, Gamepad_t *gamepad);
} // namespace manual_ctrl

namespace auto_ctrl {
/**
 * @brief 初始化自动控制线程
 *
 * 该函数用于初始化自动控制线程，并启动自动控制路径函数。
 *
 * @param[in] robot 指向机器人的指针，用于线程函数中操作机器人
 * @param[in] entry 指向线程入口函数的指针，该函数将在线程启动时被调用
 */
void init(Robot_t *robot, void (*entry)(void *parameter));

/**
 * @brief 等待机器人进入自动操作状态
 *
 * 该函数持续检查机器人的控制状态和自动子状态，直到满足启动条件。
 * 一旦满足条件，根据预定义的路径执行相应的操作。
 */
void WaitStart();

/**
 * @brief 自动阶段完成后的清理操作
 *
 * 该函数在自动阶段完成后调用，执行必要的清理操作。
 */
void Finish();

// void AutoControl_SetSpeed(Robot_t *robot, float linear_x, float linear_y,
// float angular_z, float delay_s); void AutoControl_SetPosition(Robot_t *robot,
// float x, float y, float yaw); void AutoControl_VisualFindBall(Robot_t *robot,
// uint8_t color);
} // namespace auto_ctrl

/**
 * @brief 初始化函数
 *
 * 该函数在机器人启动时执行一次，用于初始化机器人的控制模式、底盘参数、PID控制器、电机和舵机对象。
 * 它还设置机器人的初始状态，包括控制状态、底盘模式、IMU保持状态以及手动和自动子状态。
 *
 * @note
 *    - 根据机器人的底盘类型（Omni, Three-wheel Omni, Two-wheel Differential,
 * Four-wheel Differential）初始化底盘运动学。
 *    - 创建并初始化电机和舵机对象，并设置默认的PID参数。
 *    - 启动音频播放，播放启动音乐。
 *    - 初始化自动控制线程。
 */
void setup() {
  /* 获取Robot使用的控制对象指针 */
  motor_port = motor::PortPointer(); // 获取电机端口指针
  servo_port = servo::PortPointer(); // 获取舵机端口指针
  gamepad = usb::GamepadPointer();   // 获取手柄指针

  /* 初始化Robot控制模式和底盘参数 */
  robot.ctrl_state =
      STARTUP_STATE; // 设置机器人控制状态为启动状态	STARTUP_STATE 自动模式
  robot.chassis_mode = SAFE_MODE;        // 设置底盘模式为安全模式
  robot.imu_hold = true;                 // 启用IMU保持功能
  robot.manual_sub_state = GENERAL_CTRL; // 设置手动子状态为通用控制
  robot.auto_sub_state = WAIT_START;     // 设置自动子状态为等待启动

  /* 根据底盘类型初始化底盘运动学 */
#if ROBOT_CHASSIS == ROBOT_CHASSIS_OMNI
  // 如果是四轮全向底盘，初始化四轮全向底盘运动学
  chassis = new ChassisKinematics(Chassis::FOUR_WHEEL_OMNI);
  chassis->init(WHEELS_DIAMETER, OMNI4_WHEELS_TRACK, OMNI4_WHEELS_BASE,
                HUB_MOTOR_MAX_SPEED);
#elif ROBOT_CHASSIS == ROBOT_CHASSIS_THREE_WHEEL_OMNI
  // 如果是三轮全向底盘，初始化三轮全向底盘运动学
  chassis = new ChassisKinematics(Chassis::THREE_WHEEL_OMNI);
  chassis->init(WHEELS_DIAMETER, OMNI3_CENTER_DISTANCE, HUB_MOTOR_MAX_SPEED);
#elif ROBOT_CHASSIS == ROBOT_CHASSIS_TWO_DIFF
  // 如果是两轮差速底盘，初始化两轮差速底盘运动学
  chassis = new ChassisKinematics(Chassis::TOW_WHEEL_DIFFERENTIAL);
  chassis->init(WHEELS_DIAMETER, DIFF_WHEELS_TRACK, 0, HUB_MOTOR_MAX_SPEED);
#elif ROBOT_CHASSIS == ROBOT_CHASSIS_FOUR_DIFF
  // 如果是四轮差速底盘，初始化四轮差速底盘运动学
  chassis = new ChassisKinematics(Chassis::FOUR_WHEEL_DIFFERENTIAL);
  chassis->init(WHEELS_DIAMETER, DIFF_WHEELS_TRACK, DIFF_WHEELS_BASE,
                HUB_MOTOR_MAX_SPEED);
#endif

  // 初始化PID控制器
  position_x_pid.Init(&robot.pid_pos_x.input, &robot.pid_pos_x.output,
                      &robot.pid_pos_x.setpoint, 0.95, 0, 0,
                      _PID_CD_DIRECT); // 初始化X轴位置PID控制器
  position_x_pid.SetSampleTime(1000 / PID_CALC_FREQUENCY); // 设置采样时间
  position_x_pid.SetOutputLimits(-CHASSIS_MAX_LINEAR_SPEED,
                                 CHASSIS_MAX_LINEAR_SPEED); // 设置输出限制
  position_x_pid.SetMode(_PID_MODE_AUTOMATIC);              // 设置PID模式为自动

  position_y_pid.Init(&robot.pid_pos_y.input, &robot.pid_pos_y.output,
                      &robot.pid_pos_y.setpoint, 0.95, 0, 0,
                      _PID_CD_DIRECT); // 初始化Y轴位置PID控制器
  position_y_pid.SetSampleTime(1000 / PID_CALC_FREQUENCY); // 设置采样时间
  position_y_pid.SetOutputLimits(-CHASSIS_MAX_LINEAR_SPEED,
                                 CHASSIS_MAX_LINEAR_SPEED); // 设置输出限制
  position_y_pid.SetMode(_PID_MODE_AUTOMATIC);              // 设置PID模式为自动

  yaw_pid.Init(&robot.pid_yaw.input, &robot.pid_yaw.output,
               &robot.pid_yaw.setpoint, 0.08, 0, 0,
               _PID_CD_DIRECT);                     // 初始化偏航角PID控制器
  yaw_pid.SetSampleTime(1000 / PID_CALC_FREQUENCY); // 设置采样时间
  yaw_pid.SetOutputLimits(-CHASSIS_MAX_ANGULAR_SPEED,
                          CHASSIS_MAX_ANGULAR_SPEED); // 设置输出限制
  yaw_pid.SetMode(_PID_MODE_AUTOMATIC);               // 设置PID模式为自动

  /* 创建电机控制对象 */
  robot.motor[0] =
      new HubMotor(motor_port, LEFT_FRONT_MOTOR_ID); // 创建左前电机对象
  robot.motor[1] =
      new HubMotor(motor_port, RIGHT_FRONT_MOTOR_ID); // 创建右前电机对象
  robot.motor[2] =
      new HubMotor(motor_port, LEFT_BACK_MOTOR_ID); // 创建左后电机对象
  robot.motor[3] =
      new HubMotor(motor_port, RIGHT_BACK_MOTOR_ID); // 创建右后电机对象
  robot.motor[0]->setPid(1, 20, 0.8); // 设置左前电机PID参数 24V 0.3 12V 0.8
  robot.motor[1]->setPid(1, 20, 0.8); // 设置右前电机PID参数 24V 0.3 12V 0.8
  robot.motor[2]->setPid(1, 20, 0.8); // 设置左后电机PID参数 24V 0.3 12V 0.8
  robot.motor[3]->setPid(1, 20, 0.8); // 设置右后电机PID参数 24V 0.3 12V 0.8

  /* 创建舵机对象 */
  // STS 舵机创建方式 new Servo(servo_port, 1)
  // SCS 舵机创建方式 new Servo(servo_port, 1, ServoType::SCS,135,-135)
  robot.servo[0] =
      new Servo(servo_port, 1, ServoType::SCS, 135, -135); // 创建SCS舵机对象
  robot.servo[1] =
      new Servo(servo_port, 2, ServoType::SCS, 135, -135); // 创建SCS舵机对象
  robot.servo[2] =
      new Servo(servo_port, 3, ServoType::SCS, 135, -135); // 创建SCS舵机对象
  robot.servo[0]->setAngle(30); // 设置舵机0的角度为-60度（夹爪张开或框抬起）
  robot.servo[0]->setAcceleration(150); // 设置舵机0的加速度为150
  robot.servo[0]->setAngle(30);  // 设置舵机0的角度为-60度（夹爪张开或框抬起）
  robot.servo[1]->setAngle(-30); // 设置舵机1的角度为124度（夹爪张开或框抬起）
  robot.servo[1]->setAcceleration(150); // 设置舵机1的加速度为150
  robot.servo[1]->setAngle(-30); // 设置舵机1的角度为124度（夹爪张开或框抬起）
  robot.servo[2]->setAngle(-30); // 设置舵机2的角度为108度（云台pitch）
  robot.servo[2]->setAcceleration(150); // 设置舵机2的加速度为75
  robot.servo[2]->setAngle(-30);        // 设置舵机2的角度为108度（云台pitch）

  /* 机器人启动 */
  audio::playMusic("Startup", false); // 播放启动音乐
  robot.auto_path = 1;                // 设置自动路径为1
  delay(250);                         // 延迟250毫秒，等待机器人其他设备稳定

  auto_ctrl::init(&robot, auto_mode_path); // 初始化自动控制线程
}

/**
 * @brief 主循环函数
 *
 * 该函数是机器人程序的主循环，持续执行以下操作：
 * 1. 更新机器人传感器数据。
 * 2. 根据手柄按钮切换机器人的控制状态。
 * 3. 根据控制状态执行手动控制或自动控制逻辑。
 * 4. 计算PID控制量并控制底盘运动。
 *
 * @note
 *    - 在比赛中，手柄切换控制状态的功能可能不需要。
 *    - 延迟1毫秒以防止CPU占用过高。
 */
void loop() {
  // 轮子报警清除
  motor_reset();
  // 更新机器人传感器数据
  robot_sensor_update(&robot, gamepad);

  // 切换机器人控制状态
  robot_ctrl_state(&robot,
                   IS_BUTTON_CLICKED(gamepad->buttons, robot.last_buttons,
                                     GAMEPAD_XBOX_BUTTON_HOME));

  // 判断是否为手动控制状态
  if (robot.ctrl_state == MANUAL_OPERATION) {
    // 处理手动控制子状态的切换，通过按下特定按钮
    manual_ctrl::SubCtrlState(
        &robot, IS_BUTTON_CLICKED(gamepad->buttons, robot.last_buttons,
                                  GAMEPAD_XBOX_BUTTON_MINUS));

    // 根据不同的手动子状态执行相应的控制函数
    switch (robot.manual_sub_state) {
    case GENERAL_CTRL: // 如果处于通用控制状态，执行相关功能
      manual_ctrl::General(&robot, gamepad);
      break;

    case GENERAL_CTRL2: // 如果处于第二种通用控制状态，执行对应功能
      manual_ctrl::General_2(&robot, gamepad);
      break;

    default:
      // 如果没有匹配的子状态，什么都不做
      break;
    }

    // 基于手柄输入进行底盘控制（运动控制）
    manual_ctrl::ChassisSpeed(&robot, gamepad);
  }
  // 判断是否为自动操作状态
  else if (robot.ctrl_state == AUTO_OPERATION) {
    // 切换自动模式路径，适用于演示
    manual_ctrl::AutoSwitch(&robot, gamepad);
  }

  // 计算机器人的PID控制（位置和姿态控制）
  robot_pid_calc(&robot, &position_x_pid, &position_y_pid, &yaw_pid);

  // 如果不是调试状态，则进行底盘控制
  if (robot.ctrl_state != DEBUG_STATE) {
    robot_chassis_ctrl(&robot, chassis); // 控制底盘的运动
  }

  // 延迟1毫秒，控制循环频率
  delay(1);
}

/**
 * @brief 自动阶段运行控制路径函数
 *
 * 该函数在自动控制模式下运行，负责控制机器人执行预定义的路径。
 * 它持续调用 `auto_ctrl::WaitStart()` 函数，等待启动条件满足后执行相应的操作。
 *
 * @note
 *    - 该函数在自动控制线程中被调用。
 *    - `auto_ctrl::Finish()`
 * 函数在自动阶段完成后被调用，用于执行必要的清理操作。
 */
void auto_mode_path(void *parameter) {
  while (1) {
    // 等待自动控制启动条件满足
    auto_ctrl::WaitStart();

    // 执行自动控制完成后的清理操作
    auto_ctrl::Finish();
  }
}
void motor_reset() {
  if (robot.ctrl_state != DEBUG_STATE) {
    for (int i = 0; i < 4; ++i) {
      robot.motor[i]->clearStatus();
      delay(1);
    }
  }
}
