#include "robot.h"
#include "robot_function.h"

namespace manual_ctrl
{

    /**
     * @brief 使用手柄的菜单键切换子模式
     */
    void SubCtrlState(Robot_t *robot, bool button_clicked)
    {
        if (button_clicked)
        {
            robot->manual_sub_state++;

            if (robot->manual_sub_state >= NUM_SUB_STATES)
            {
                robot->manual_sub_state = GENERAL_CTRL;
            }

            /* 这里做个LED提示 */
            if (robot->manual_sub_state == GENERAL_CTRL)
            {
                if (robot->team_color == RED)
                    io::setLedColor(LED_COLOR_RED);
                else if (robot->team_color == BLUE)
                    io::setLedColor(LED_COLOR_BLUE);
            }
        }
    }

    /**
     * @brief 通过手柄按钮控制不同外设输出
     */
    int a = 0;
    int b;
    void General(Robot_t *robot, Gamepad_t *gamepad)
    {
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_A))
        {
        }
        else if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_B))
        {
            if (a == 0)
            {
                robot->servo[0]->setAngle(-60); // 爪子合拢（框放下）的角度
                robot->servo[1]->setAngle(124); // 爪子合拢（框放下）的角度
                a = 1;
            }
            else
            {
                robot->servo[0]->setAngle(-40); // 爪子张开（框抬起）的角度
                robot->servo[1]->setAngle(104); // 爪子张开（框抬起）的角度
                a = 0;
            }
        }
        else if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_X))
        {
            robot->motor[0]->clearStatus();
            robot->motor[1]->clearStatus();
            robot->motor[2]->clearStatus();
            robot->motor[3]->clearStatus();
        }
        else if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_LB))
        {
            robot->SGs.SG5_PWR = !robot->SGs.SG5_PWR;
            robot->SGs.SG5_IO = !robot->SGs.SG5_IO;
        }
        else if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_RB))
        {
            robot->SGs.SG6_PWR = !robot->SGs.SG6_PWR;
            robot->SGs.SG6_IO = !robot->SGs.SG6_IO;
        }
#ifndef SWITCHGEAR_4_USE_PWM
        else if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_Y))
        {
            robot->SGs.SG4_PWR = !robot->SGs.SG4_PWR;
            robot->SGs.SG4_IO = !robot->SGs.SG4_IO;
        }
#else
        {
            io::setSG4(1, gamepad->rt); //*涵道手动大小  io::setSG4(1,int((gamepad->rt)/2)) 风力减小两倍*//
        }
#endif
    }

    /**
     * @brief 通过手柄按钮控制不同外设输出
     */

    void General_2(Robot_t *robot, Gamepad_t *gamepad)
    {
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_A))
        {
            robot->servo[0]->setAngle(0);
        }
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_B))
        {
            robot->servo[0]->setAngle(30);
        }
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_X))
        {
            robot->servo[0]->setAngle(60);
        }
    }

    /**
     * @brief 手柄切换模式
     */
    void AutoSwitch(Robot_t *robot, Gamepad_t *gamepad)
    {
        if (robot->auto_sub_state != WAIT_START)
            return;

        /* 手柄按键ABXY设置路径 */
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_A))
        {
            robot->auto_path = 1;
        }
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_B))
        {
            robot->auto_path = 2;
        }
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_X))
        {
            robot->auto_path = 3;
        }
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_Y))
        {
            robot->auto_path = 4;
        }

        /* 手柄遥感按下，启动自动代码 */
        if (IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_THUMBL) || IS_BUTTON_CLICKED(gamepad->buttons, robot->last_buttons, GAMEPAD_XBOX_BUTTON_THUMBR))
        {
            robot->auto_sub_state = START;
        }
    }

    /**
     * @brief 手柄底盘速度控制
     */
    void ChassisSpeed(Robot_t *robot, Gamepad_t *gamepad)
    {
        if (robot->chassis_mode != SPEED_MODE)
            robot->chassis_mode = SPEED_MODE;
        robot->set_vel.linear_x = fmap(gamepad->y, INT16_MIN, INT16_MAX, -CHASSIS_MAX_LINEAR_SPEED, CHASSIS_MAX_LINEAR_SPEED);     // m/s
        robot->set_vel.linear_y = -fmap(gamepad->x, INT16_MIN, INT16_MAX, -CHASSIS_MAX_LINEAR_SPEED, CHASSIS_MAX_LINEAR_SPEED);    // m/s
        robot->set_vel.angular_z = -fmap(gamepad->z, INT16_MIN, INT16_MAX, -CHASSIS_MAX_ANGULAR_SPEED, CHASSIS_MAX_ANGULAR_SPEED); // rad/s

        if (gamepad->hat & GAMEPAD_XBOX_HAT_UP)
        {
            robot->set_vel.linear_x = 0.5; // 十字按键X速度
        }
        else if (gamepad->hat & GAMEPAD_XBOX_HAT_DOWN)
        {
            robot->set_vel.linear_x = -0.5; // 十字按键X速度
        }

#if (ROBOT_CHASSIS == ROBOT_CHASSIS_OMNI) || (ROBOT_CHASSIS == ROBOT_CHASSIS_THREE_WHEEL_OMNI)
        if (gamepad->hat & GAMEPAD_XBOX_HAT_LEFT)
        {
            robot->set_vel.linear_y = 0.5; // 十字按键Y速度
        }
        else if (gamepad->hat & GAMEPAD_XBOX_HAT_RIGHT)
        {
            robot->set_vel.linear_y = -0.5; // 十字按键Y速度
        }
#elif (ROBOT_CHASSIS == ROBOT_CHASSIS_TWO_DIFF) || (ROBOT_CHASSIS == ROBOT_CHASSIS_FOUR_DIFF)
        if (gamepad->hat & GAMEPAD_XBOX_HAT_LEFT)
        {
            robot->set_vel.angular_z = 0.5; // 十字按键YAW速度
        }
        else if (gamepad->hat & GAMEPAD_XBOX_HAT_RIGHT)
        {
            robot->set_vel.angular_z = -0.5; // 十字按键YAW速度
        }
#endif

        /* 开启坐标保持时，将旋转速度转变为角度值 */
        if (robot->imu_hold)
        {
            float tmp = degrees(robot->set_vel.angular_z);
            constrain(tmp, -degrees(CHASSIS_MAX_ANGULAR_SPEED), degrees(CHASSIS_MAX_ANGULAR_SPEED));
            robot->target_pos.yaw = LIMIT_ANGLE_180(robot->current_pos.yaw + tmp);
        }
    }

}