#include "robot_function.h"
#include "easy_math.h"

/**
 * @brief 加速函数, 是控制更平滑
 *
 * @param speed_x
 * @param speed_y
 */
void do_acc(float *speed_x, float *speed_y) {
  static float last_speed_x = 0;
  static float last_speed_y = 0;

  float acc_x = 0;
  float acc_y = 0;
  float acc_whole = 0;
  float sin_x = 0;
  float sin_y = 0;
  float tmp_speed_x = *speed_x;
  float tmp_speed_y = *speed_y;

  acc_x = tmp_speed_x - last_speed_x;
  acc_y = tmp_speed_y - last_speed_y;
  acc_whole = sqrt(acc_x * acc_x + acc_y * acc_y);

  sin_x = acc_x / acc_whole;
  sin_y = acc_y / acc_whole;

  if (acc_whole > (CHASSIS_MAX_ACC)) {
    acc_whole = (CHASSIS_MAX_ACC);
    acc_x = acc_whole * sin_x;
    acc_y = acc_whole * sin_y;
    *speed_x = last_speed_x + acc_x;
    *speed_y = last_speed_y + acc_y;
  }
  // printf("speed_x %.2f speed_y %.2f\r\n", (*speed_x), (*speed_y));

  last_speed_x = *speed_x;
  last_speed_y = *speed_y;
}

/**
 * @brief
 *
 * @param robot
 * @param chassis
 */
void do_chassis_safe_ctrl(Robot_t *robot, ChassisKinematics *chassis) {
  if (chassis->type_ == Chassis::FOUR_WHEEL_OMNI ||
      chassis->type_ == Chassis::FOUR_WHEEL_DIFFERENTIAL) {
    robot->motor[0]->setSpeed(0);
    robot->motor[1]->setSpeed(0);
    robot->motor[2]->setSpeed(0);
    robot->motor[3]->setSpeed(0);
  } else if (chassis->type_ == Chassis::THREE_WHEEL_OMNI) {
    robot->motor[0]->setSpeed(0);
    robot->motor[1]->setSpeed(0);
    robot->motor[2]->setSpeed(0);
  } else if (chassis->type_ == Chassis::TOW_WHEEL_DIFFERENTIAL) {
    robot->motor[0]->setSpeed(0);
    robot->motor[1]->setSpeed(0);
  }
}

void do_chassis_speed_ctrl(Robot_t *robot, ChassisKinematics *chassis) {
  static ChassisVelocities vel;
  vel.linear_x = robot->set_vel.linear_x;
  vel.linear_y = robot->set_vel.linear_y;
  if (robot->imu_hold) {
    vel.angular_z = robot->pid_yaw.output;
  } else {
    vel.angular_z = robot->set_vel.angular_z;
  }

  // do_acc(&(vel.linear_x), &(vel.linear_y));
  chassis->applyMotionCommand(&vel);
  if (chassis->type_ == Chassis::FOUR_WHEEL_OMNI ||
      chassis->type_ == Chassis::FOUR_WHEEL_DIFFERENTIAL) {
    robot->motor[0]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_FRONT_WHEEL]);
    robot->motor[1]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_FRONT_WHEEL]);
    robot->motor[2]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_BACK_WHEEL]);
    robot->motor[3]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_BACK_WHEEL]);
  } else if (chassis->type_ == Chassis::THREE_WHEEL_OMNI) {
    robot->motor[0]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_WHEEL]);
    robot->motor[1]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_WHEEL]);
    robot->motor[2]->setSpeed(chassis->wheels_rpm_[Chassis::BACK_WHEEL]);
  } else if (chassis->type_ == Chassis::TOW_WHEEL_DIFFERENTIAL) {
    robot->motor[0]->setSpeed(chassis->wheels_rpm_[Chassis::LEFT_WHEEL]);
    robot->motor[1]->setSpeed(chassis->wheels_rpm_[Chassis::RIGHT_WHEEL]);
  }
}

void do_chassis_position_ctrl(Robot_t *robot, ChassisKinematics *chassis) {
  float linear_x = robot->pid_pos_x.output;
  float linear_y = robot->pid_pos_y.output;
  float angular_z = robot->pid_yaw.output;

  float sin_yaw = sin(radians(robot->current_pos.yaw));
  float cos_yaw = cos(radians(robot->current_pos.yaw));

  robot->set_vel.linear_x = cos_yaw * linear_x + sin_yaw * linear_y;
  robot->set_vel.linear_y = -sin_yaw * linear_x + cos_yaw * linear_y;
  robot->set_vel.angular_z = angular_z;

  do_chassis_speed_ctrl(robot, chassis);
}

void do_chassis_detect_speeds(Robot_t *robot, ChassisKinematics *chassis) {
  float motor_speed[4];
  if (chassis->type_ == Chassis::FOUR_WHEEL_OMNI ||
      chassis->type_ == Chassis::FOUR_WHEEL_DIFFERENTIAL) {
    motor_speed[Chassis::LEFT_FRONT_WHEEL] = robot->motor[0]->getSpeed();
    motor_speed[Chassis::RIGHT_FRONT_WHEEL] = robot->motor[1]->getSpeed();
    motor_speed[Chassis::LEFT_BACK_WHEEL] = robot->motor[2]->getSpeed();
    motor_speed[Chassis::RIGHT_BACK_WHEEL] = robot->motor[3]->getSpeed();
  } else if (chassis->type_ == Chassis::THREE_WHEEL_OMNI) {
    motor_speed[Chassis::LEFT_WHEEL] = robot->motor[0]->getSpeed();
    motor_speed[Chassis::RIGHT_WHEEL] = robot->motor[1]->getSpeed();
    motor_speed[Chassis::BACK_WHEEL] = robot->motor[2]->getSpeed();
  } else if (chassis->type_ == Chassis::TOW_WHEEL_DIFFERENTIAL) {
    motor_speed[Chassis::LEFT_WHEEL] = robot->motor[0]->getSpeed();
    motor_speed[Chassis::RIGHT_WHEEL] = robot->motor[1]->getSpeed();
  }

  chassis->detectMovementSpeeds(motor_speed);
  chassis->updateOdom(radians(robot->current_pos.yaw));
  robot->current_pos.x = chassis->odom_.x_pos;
  robot->current_pos.y = chassis->odom_.y_pos;
}

/**
 * @brief 检测手柄是否链接, 手柄连接与拔出播放提示音
 */
bool robot_gamepad_is_connect() {
  static bool conn_state = false;
  if (usb::isConnected() != conn_state) {
    if (conn_state == false) {
      audio::playMusic("Connect", false);
      conn_state = true;
    } else {
      audio::playMusic("Disconnect", false);
      conn_state = false;
    }
  }
  return conn_state;
}

/**
 * @brief 机器人控制状态切换
 *
 * @param robot
 * @param button_clicked 用于切换的按键，可以是实体按键也可以是手柄按键
 */
void robot_ctrl_state(Robot_t *robot, bool button_clicked) {
  static uint16_t last_state = STARTUP_STATE;

  if (button_clicked) {
    switch (robot->ctrl_state) {
    case STARTUP_STATE:
      robot->ctrl_state = MANUAL_OPERATION;
      break;
    case MANUAL_OPERATION:
      robot->ctrl_state = AUTO_OPERATION;
      break;
    case AUTO_OPERATION:
      robot->ctrl_state = STARTUP_STATE;
      break;
    default:
      break;
    }
  }

  /* 手柄未连接的时候进入保护状态（比赛中可能不需要） */
  if (!robot_gamepad_is_connect() && (robot->ctrl_state == MANUAL_OPERATION)) {
    robot->ctrl_state = STARTUP_STATE;
  }

  if (last_state != robot->ctrl_state) {
    /* 切换状态时复位参数，使机器人停下来 */
    robot_reset(robot);

    /* 做一个状态切换的提示 */
    switch (robot->ctrl_state) {
    case STARTUP_STATE:
      io::setLedColor(LED_COLOR_OFF);
      break;
    case MANUAL_OPERATION:
      audio::playMusic("ManualCtrl", false);
      if (robot->team_color == RED)
        io::setLedColor(LED_COLOR_RED);
      else if (robot->team_color == BLUE)
        io::setLedColor(LED_COLOR_BLUE);
      break;
    case AUTO_OPERATION:
      audio::playMusic("AutoCtrl", false);
      io::setLedColor(LED_COLOR_GREEN);
      break;
    case DEBUG_STATE:
      io::setLedColor(LED_COLOR_OFF);
      break;
    }
  }
  last_state = robot->ctrl_state;
}

void robot_sensor_update(Robot_t *robot, Gamepad_t *gamepad) {
  static struct switchgear last_SGs = {0};
  static float battery_ms = millis();

  robot->last_buttons = robot->buttons;
  robot->buttons = gamepad->buttons;

  /* 小于4秒读IO选择队伍并使用LED显示 */
  if (millis() < 4000) {
    if (io::getIN1()) {
      robot->team_color = RobotColor::RED;
      io::setLedColor(LED_COLOR_RED);
    } else {
      robot->team_color = RobotColor::BLUE;
      io::setLedColor(LED_COLOR_BLUE);
    }
  }
  if (*(uint16_t *)(&last_SGs) != *(uint16_t *)(&robot->SGs)) {
    io::setSG1(robot->SGs.SG1_PWR, robot->SGs.SG1_IO);
    io::setSG2(robot->SGs.SG2_PWR, robot->SGs.SG2_IO);
    io::setSG3(robot->SGs.SG3_PWR, robot->SGs.SG3_IO);
    io::setSG4(robot->SGs.SG4_PWR, robot->SGs.SG4_IO);
    io::setSG5(robot->SGs.SG5_PWR, robot->SGs.SG5_IO);
    io::setSG6(robot->SGs.SG6_PWR, robot->SGs.SG6_IO);
  }

  if (millis() - battery_ms > 500) {
    battery_ms = millis();
    robot->motor[0]->readStatus();
    if (robot->motor[0]->comm_result_ == HM_SUCCESS) {
      robot->battery_voltage = robot->motor[0]->getPowerVoltage();
    } else {
      robot->battery_voltage = 0;
    }
#ifdef LOW_BATTERY_ALARM_ENABLED
    if (robot->battery_voltage <= 10.6) {
      // audio::playMusic("BattCharge", true);
    }
#endif
  }

  last_SGs = robot->SGs;

  imu::getAngle(&robot->current_pos.roll, &robot->current_pos.pitch,
                &robot->current_pos.yaw);
}

/**
 * @brief 机器人位置闭环
 *
 * @param robot
 * @param x_pid
 * @param y_pid
 * @param yaw_pid
 */
void robot_pid_calc(Robot_t *robot, PID *x_pid, PID *y_pid, PID *yaw_pid) {
  // 设置pid计算值
  robot->pid_yaw.input = robot->current_pos.yaw;
  robot->pid_yaw.setpoint = robot->target_pos.yaw;
  robot->pid_pos_x.input = robot->current_pos.x;
  robot->pid_pos_x.setpoint = robot->target_pos.x;
  robot->pid_pos_y.input = robot->current_pos.y;
  robot->pid_pos_y.setpoint = robot->target_pos.y;

  // yaw轴过换向点保持角度连续
  if (robot->pid_yaw.setpoint - robot->pid_yaw.input > 180) {
    robot->pid_yaw.setpoint -= 360;
  } else if (robot->pid_yaw.setpoint - robot->pid_yaw.input < -180) {
    robot->pid_yaw.setpoint += 360;
  }

  yaw_pid->Compute();
  x_pid->Compute();
  y_pid->Compute();
}

void robot_chassis_ctrl(Robot_t *robot, ChassisKinematics *chassis) {
  switch (robot->chassis_mode) {
  case SAFE_MODE:
    do_chassis_safe_ctrl(robot, chassis);
    break;
  case SPEED_MODE:
    do_chassis_speed_ctrl(robot, chassis);
    break;
  case POSTION_MODE:
    do_chassis_position_ctrl(robot, chassis);
    break;
  default:
    break;
  }
  do_chassis_detect_speeds(robot, chassis);
}

void robot_reset(Robot_t *robot) {
  robot->manual_sub_state = 0;
  robot->auto_sub_state = 0;
  robot->chassis_mode = SAFE_MODE;
  robot->set_vel.linear_x = 0;
  robot->set_vel.linear_y = 0;
  robot->set_vel.angular_z = 0;
  robot->target_pos.x = robot->current_pos.x;
  robot->target_pos.y = robot->current_pos.y;
  robot->target_pos.z = robot->current_pos.z;
  robot->target_pos.pitch = robot->current_pos.pitch;
  robot->target_pos.roll = robot->current_pos.roll;
  robot->target_pos.yaw = robot->current_pos.yaw;

  /* 舵机等机构复位 */
  // robot->servo[0]->setAngle(0);
}
