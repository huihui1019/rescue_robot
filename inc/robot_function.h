#ifndef __ROBOT_FUNCTION_H__
#define __ROBOT_FUNCTION_H__

#include "robot.h"
#include "robot_driver.h"
#include "chassis_kinematics.hpp"

#define IS_BUTTON_PRESSED(current_buttons, button_mask) (current_buttons & button_mask)
#define IS_BUTTON_CLICKED(current_buttons, last_buttons, button_mask) \
    ((current_buttons & button_mask) &&!(last_buttons & button_mask))

void do_acc(float *speed_x, float *speed_y);
void do_chassis_safe_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_speed_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_position_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void do_chassis_detect_speeds(Robot_t *robot, ChassisKinematics *chassis);

bool robot_gamepad_is_connect();
void robot_ctrl_state(Robot_t *robot, bool button_clicked);
void robot_sensor_update(Robot_t *robot, Gamepad_t *gamepad);
void robot_pid_calc(Robot_t *robot, PID *pos_x, PID *pos_y, PID *yaw);
void robot_chassis_ctrl(Robot_t *robot, ChassisKinematics *chassis);
void robot_reset(Robot_t *robot);



#endif /* __ROBOT_FUNCTION_H__ */