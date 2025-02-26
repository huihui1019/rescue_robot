#include "robot.h"
#include "robot_function.h"

namespace auto_ctrl {
#define AUTO_THREAD_PRIORITY 1      // 自动控制线程的优先级
#define AUTO_THREAD_STACK_SIZE 2048 // 自动控制线程的堆栈大小（单位：字节）
#define AUTO_THREAD_TIMESLICE 5     // 自动控制线程的时间片（单位：毫秒）
#define M_PI 3.14159265358979323846 // 定义圆周率常数
// 角度与弧度转换宏
#define DEGREE_TO_RADIAN(degree) ((degree) * M_PI / 180.0)
#define RADIAN_TO_DEGREE(radian) ((radian) * 180.0 / M_PI)
static rt_thread_t auto_thread = RT_NULL; // 定义自动控制线程句柄，初始为NULL
static Robot_t *robot_ = NULL;            // 定义机器人指针，初始为NULL
struct position *current;                 // 定义当前位置结构体指针
bool VisualFindBall(Robot_t *robot, uint8_t color, uint8_t pd); // 找球函数
void SetSpeed(
    float linear_x, float linear_y, float angular_z,
    float delay_s); // 跑速度的函数（x上的速度，y上速度，z上旋转速度，时间）
                    // 建议每次只给一个方向上的速度和时间
void MovePosition(float x, float y,
                  float yaw); // 跑里程计点位（x位置，y位置，yaw轴位置）
void Turn_angle();            // 控制底盘转向（1.05，-0.95）方向
void Turn_angle2();           // 控制底盘转向（1.05，-0.35）方向
void corner_angle();          // 控制整车放球时的姿态
void PART1();                 // 自动逻辑1（不带低头检测）#官方案例路线
void PART2();   // 自动逻辑2（带低头检测）#使用此路线需把云台架高至合适高度
void find1();   // 通过里程计跑到find1位置(左下角)
void find2();   // 通过里程计跑到find2位置(右下角)
void find3();   // 通过里程计跑到find3位置(右上角)
void find4();   // 通过里程计跑到find4位置(左上角)
void go_home(); // 通过里程计大致跑家函数
void only_one(); // 可以读舵机角度查看是否堵转，检测是否为多个（对机构有严格要求）
bool certain_home(Robot_t *robot, uint8_t color,
                  uint8_t pd); // 判断安全区是否在画面中心
int certain_ball(
    Robot_t *robot, uint8_t color,
    uint8_t pd); // 判断是否有球函数 #使用此函数需把云台架高至合适高度
int notfind = 0; // 未寻找到球的次数
bool pd = 0;     // 判断找球反馈超时还是找到球的一个变量
int pd2 = 0;     // 判断找球反馈超时还是找到球的一个变量
int mine = 0;    // 本方球寻找次数的变量   mine=2开始寻找除对方球的所有球
int a;           // 只要什么颜色球
int b;           // 除了什么颜色球
int c;           // 什么颜色安全区

/*初始化函数 init，用于创建并启动线程*/
void init(Robot_t *robot, void (*entry)(void *parameter)) {
  auto_thread = rt_thread_create("auto", entry, robot, AUTO_THREAD_STACK_SIZE,
                                 AUTO_THREAD_PRIORITY, AUTO_THREAD_TIMESLICE);
  if (auto_thread != RT_NULL)
    rt_thread_startup(auto_thread);
  robot_ = robot;
}

/**
 * @brief 等待开始
 */
void WaitStart() {
  /*机器人的自动子状态现在处于等待开始的状态*/
  if (robot_->auto_sub_state != WAIT_START) {
    robot_->auto_sub_state = WAIT_START;
  }
  while (1) {
    /*是否满足两个条件1.机器人处于自动状态2.机器人的自动子状态现在处于开始*/
    if (robot_->ctrl_state == AUTO_OPERATION &&
        robot_->auto_sub_state == START) //
    {
      /*通过switch(robot.auto_path)选择路径*/
      switch (robot.auto_path) {
      case 1:
        robot.servo[0]->setAngle(-16); // 爪子合拢（框放下）的角度
        robot.servo[1]->setAngle(13);  // 爪子合拢（框放下）的角度
        robot.imu_hold = true;         // 陀螺仪保持
        SetSpeed(0.7, 0, 0, 1.25);     // 向前冲
        robot.imu_hold = false;        // 陀螺仪不保持
        // SetSpeed(0, 0,-7,1.25);//扫尾*1 （把球（物料）撞散）
        // SetSpeed(0, 0,7,1.25);//扫尾*2（把球（物料）撞散）
        MovePosition(robot.current_pos.x, robot.current_pos.y,
                     0); // 原地旋转到0度
        // robot.servo[0]->setAngle(30);//爪子张开（框抬起）的角度
        // robot.servo[1]->setAngle(-30);//爪子张开（框抬起）的角度
        // SetSpeed(-0.6, 0, 0,0.5);//向后退
        // robot.servo[0]->setAngle(-16);//爪子合拢（框放下）的角度
        // robot.servo[1]->setAngle(13);//爪子合拢（框放下）的角度
        // SetSpeed(0, 0,-0.8,0.75);//向右旋转一定方向
        PART1(); // 自动路径1
        break;
      case 2:
        robot.servo[0]->setAngle(-5); // 爪子合拢（框放下）的角度
        robot.imu_hold = true;        // 陀螺仪保持
        SetSpeed(0.6, 0, 0, 0.85);    // 向前冲
        PART1();                      // 自动路径1
        break;
      case 3:
        robot.servo[0]->setAngle(-5); // 爪子合拢（框放下）的角度
        robot.imu_hold = true;        // 陀螺仪保持
        SetSpeed(0.6, 0, 0, 1.4);     // 向前冲
        robot.imu_hold = false;       // 陀螺仪不保持
        SetSpeed(0, 0, -7, 1.5);      // 扫尾*1（把球（物料）撞散）
        SetSpeed(0, 0, 7, 1.5);       // 扫尾*2（把球（物料）撞散）
        MovePosition(robot.current_pos.x, robot.current_pos.y,
                     0);            // 原地旋转到0度
        SetSpeed(-0.6, 0, 0, 0.5);  // 向后退
        SetSpeed(0, 0, -0.8, 0.75); // 向右旋转一定方向
        PART2();                    // 自动路径2
        break;
      case 4: // 开环自动路径
        /*此处是给你写固定路线放球的地方  #上场前必须准备的保底方案
         * 可参考find1-4移动来写*/
        break;
      }

      return; // 返回
    }
    delay(100); // 延时100ms
  }
}

/**
 * @brief //切换到结束状态
 */
void Finish() {
  robot_->auto_sub_state = FINISH; // 切换到结束状态
}

/**
 * @brief 设置机器人速度
 *
 * @param linear_x 前进直线速度。单位m/s
 * @param linear_y 左右直线速度。单位m/s
 * @param angular_z 旋转速度。单位rad/s
 * @param delay_s 执行时间。单位s
 */
void SetSpeed(float linear_x, float linear_y, float angular_z, float delay_s) {
// 定义速度模式下的超时时间，单位：毫秒 (ms)
#define _SET_SPEED_TIMEOUT 10
  // 延迟时间 (delay_s) 乘以 1000 转换为毫秒，再除以每次循环的延迟时间
  // (_SET_SPEED_TIMEOUT)
  uint16_t ticks = delay_s * 1000 / _SET_SPEED_TIMEOUT;
  // 小车进入速度模式
  robot_->chassis_mode = SPEED_MODE;
  robot_->set_vel.linear_x = linear_x;   // 小车x速度为x
  robot_->set_vel.linear_y = linear_y;   // 小车y速度为y
  robot_->set_vel.angular_z = angular_z; // 小车z速度为z
  if (robot_->imu_hold)                  // 判断是否陀螺仪保持
  {
    // 将旋转速度从弧度转换为角度
    float tmp = degrees(robot_->set_vel.angular_z);
    // 计算目标偏航角，限制在 -180 到 180 度之间
    robot_->target_pos.yaw = LIMIT_ANGLE_180(robot_->current_pos.yaw + tmp);
  }

  do {
    delay(_SET_SPEED_TIMEOUT); // 每次延迟 _SET_SPEED_TIMEOUT 毫秒
  } while (ticks--); // 循环直到 ticks 为 0
  robot_->chassis_mode = SAFE_MODE; // 小车进入安全模式
}

/**
 * @brief 通过目标位置减去现在位置判断是否在目标位置上
 * @param target 目标位置信息
 * @param current 现在位置信息
 * @param pos_tol X,Y的允许范围大小
 * @param yaw_tol YAW轴的允许范围大小
 * @return true 在允许误差范围内
 * @return false 不在允许误差范围内
 */
bool IsWithinTolerance(struct position *target, struct position *current,
                       float pos_tol = 0.03, float yaw_tol = 1) {
#if defined(ROBOT_CHASSIS_TWO_DIFF) // 两轮差速
  if (robot.target_pos.yaw == 90 or
      robot.target_pos.yaw == -90 or // 判断目标yaw角度是否为 +-90或+-270
      robot.target_pos.yaw == 270 or robot.target_pos.yaw == -270) {
    if (fabs(robot.target_pos.y - robot.current_pos.y) <
            pos_tol && // 判断目标y和现在的y是否在差值内
        fabs(robot.target_pos.yaw - robot.current_pos.yaw) <
            1) // 判断目标yaw和现在的yaw是否在差值内
    {
      return true; // 返回 true
    }
  }
  if (robot.target_pos.yaw == 180 or robot.target_pos.yaw == 0 or
      robot.target_pos.yaw == -180) // 判断目标yaw角度是否为 0 或 +-180
  {
    if (fabs(robot.target_pos.x - robot.current_pos.x) <
            pos_tol && // 判断目标x和现在的x是否在差值内
        fabs(robot.target_pos.yaw - robot.current_pos.yaw) <
            1) // 判断目标yaw和现在的yaw是否在差值内
    {
      return true; // 返回 true
    }
  }
  return false; // 返回 false
#endif
  if (fabs(target->x - current->x) <
          pos_tol && // 判断目标x和现在的x是否在差值内
      fabs(target->y - current->y) <
          pos_tol && // 判断目标y和现在的y是否在差值内
      fabs(target->yaw - current->yaw) <
          yaw_tol) // 判断目标yaw和现在的yaw是否在差值内
  {
    return true; // 返回 true
  }
  return false; // 返回 false
}
/**
 * @brief 小车的里程计模式控制函数
 * @param x x最后到的位置
 * @param y y最后到的位置
 * @param yaw 最后的yaw轴姿态
 */
void MovePosition(float x, float y, float yaw) {
  robot_->chassis_mode = POSTION_MODE;           // 小车进入里程计模式
  robot_->target_pos.x = x;                      // 目标位置x为x
  robot_->target_pos.y = y;                      // 目标位置y为y
  robot_->target_pos.yaw = LIMIT_ANGLE_180(yaw); // 目标位置yaw为yaw
  while (!IsWithinTolerance(&robot_->target_pos,
                            &robot_->current_pos)) // 判断是否到位
  {
    if (robot.ctrl_state == MANUAL_OPERATION ||
        robot.ctrl_state == STARTUP_STATE) {
      break; // 切换模式退出
    }
    delay(10); // 延时10ms
  }
  robot_->chassis_mode = SAFE_MODE; // 小车进入安全模式
}
/**
 * @brief 小车的转向函数（转向本方安全区）
 * @param home_x 安全区x的位置
 * @param home_y 安全区y的位置
 */
float home_x = 1.05;
float home_y = -0.95;
void Turn_angle() {
  // 两个点的坐标
  float x1, y1, x2, y2;
  x1 = robot_->current_pos.x; // 现在小车的X位置
  y1 = robot_->current_pos.y; // 现在小车的Y位置
  x2 = home_x;                // 我方安全区位置X
  y2 = home_y;                // 我方安全区位置Y

  // 计算两点之间的角度
  float deltaX = x2 - x1;
  float deltaY = y2 - y1;

  // 使用 atan2 计算角度，结果以弧度返回
  float angleInRadians = atan2(deltaY, deltaX);

  // 将弧度转换为度
  float angleInDegrees = RADIAN_TO_DEGREE(angleInRadians);
  robot_->chassis_mode = POSTION_MODE;          // 小车进入里程计模式
  robot_->target_pos.x = robot_->current_pos.x; // 目标X为现在X
  robot_->target_pos.y = robot_->current_pos.y; // 目标Y为现在Y
  robot_->target_pos.yaw =
      LIMIT_ANGLE_180(angleInDegrees);           // 目标YAW为转向本方安全区方向
  while (robot_->ctrl_state == AUTO_OPERATION) { // 自动模式下
    if (fabs(robot.target_pos.yaw - robot.current_pos.yaw) <
        1) { // 判断目标角度和现在角度差值绝对值是否小于1度
      break; // 退出循环
    }
    delay(10); // 延时10ms
  }
}
/**
 * @brief 小车的转向函数（转向中心靠安全区方向）
 * @param home_xx 中心靠安全区x的位置
 * @param home_yy 中心靠安全区y的位置
 */
float home_xx = 1.05;
float home_yy = -0.35;
void Turn_angle2() {
  // 两个点的坐标
  float x1, y1, x2, y2;
  x1 = robot_->current_pos.x;     // 现在小车的X位置
  y1 = robot_->current_pos.y;     // 现在小车的Y位置
  x2 = home_xx + (home_x - 1.05); // 目标点位x（场地中心位置偏本方安全区）
  y2 = home_yy + (home_y + 0.95); // 目标点位y（场地中心位置偏本方安全区）

  // 计算两点之间的角度
  float deltaX = x2 - x1;
  float deltaY = y2 - y1;

  // 使用 atan2 计算角度，结果以弧度返回
  float angleInRadians = atan2(deltaY, deltaX);

  // 将弧度转换为度
  float angleInDegrees = RADIAN_TO_DEGREE(angleInRadians);
  robot_->chassis_mode = POSTION_MODE;          // 小车进入里程计模式
  robot_->target_pos.x = robot_->current_pos.x; // 目标X为现在X
  robot_->target_pos.y = robot_->current_pos.y; // 目标Y为现在Y
  robot_->target_pos.yaw =
      LIMIT_ANGLE_180(angleInDegrees);           // 目标YAW为转向（x2，y2）方向
  while (robot_->ctrl_state == AUTO_OPERATION) { // 自动模式下
    if (fabs(robot.target_pos.yaw - robot.current_pos.yaw) <
        1) { // 判断目标角度和现在角度差值绝对值是否小于1度
      break; // 退出循环
    }
    delay(10); // 延时10ms
  }
}
/**
 * @brief 检测球的颜色
 * @param color 查找颜色
 * @param pd 0仅仅此颜色  1除了此颜色其他颜色
 * @return 1 颜色不一致
 * @return 2 颜色一致
 * @return false 未找到任何颜色
 */
int certain_ball(Robot_t *robot, uint8_t color, uint8_t pd) {
  // 定义两个存储时间的变量
  uint32_t dt = 0;                             // 数据更新时间
  uint32_t ct = millis();                      // 进入函数的初始时间
  while (robot_->ctrl_state == AUTO_OPERATION) // 自动模式下
  {
    if ((millis() - ct) > 500) // 判断现在时间与初始时间的差是否大于500
    {
      return false; // 返回 false
    }
    if (robot->ctrl_state == MANUAL_OPERATION ||
        robot->ctrl_state == STARTUP_STATE) {
      return false; // 切换模式时自动退出
    }
    // 初始化视觉检测变量
    visual_t *ball_tmp = NULL;       // 视觉反馈下发的所有数据集合
    visual_t *find_ball = NULL;      // 最后符合条件的数据集合
    uint32_t jl = 9999999;           // 定义一个距离变量为无限大即可
    for (uint8_t i = 0; i < 14; i++) // for循环遍历14次
    {
      delay(1);                      // 延时1ms
      ball_tmp = visual::getBall(i); // 获取视觉下发数据
      if (pd == 1) { // 判断是color颜色还是除color颜色以外（1不是color颜色）
        if (color != ball_tmp->type && ball_tmp->type != 1 &&
            ball_tmp->type != 2) // 判断视觉数据和color是否不同，且不等于1和2
        {
          dt = (millis() -
                ball_tmp->timestamp); // 计算现在时间与数据获取时间的插值
          if (dt < 100)               // 判断dt是否小于100ms
          {
            return 1; // 返回 1
          }
        }
        if (color == ball_tmp->type && ball_tmp->type != 1 &&
            ball_tmp->type != 2) // 判断视觉数据和color是否相同，且不等于1和2
        {
          dt = (millis() -
                ball_tmp->timestamp); // 计算现在时间与数据获取时间的插值
          if (dt < 100)               // 判断dt是否小于100ms
          {
            return 2; // 返回 2
          }
        }
      }
    }
    delay(10); // 延时10ms
  }
  return false; // 返回 false
}
/**
 * @brief 检测安全区的位置
 * @param color 查找红安全区/蓝安全区
 * @param pd 0仅仅此颜色
 * @return false 未找到安全区或安全区不在画面中心
 * @return true 找到安全区在画面中心
 */
bool certain_home(Robot_t *robot, uint8_t color, uint8_t pd) {
  // 定义两个存储时间的变量
  uint32_t dt = 0;                             // 数据更新时间
  uint32_t ct = millis();                      // 进入函数的初始时间
  while (robot_->ctrl_state == AUTO_OPERATION) // 自动模式下
  {
    if ((millis() - ct) > 1000) // 判断初始时间与现在时间的插值是否大于1秒
    {
      robot->imu_hold = false; // 陀螺仪保持关闭
      return false;            // 返回 false
    }
    if (robot->ctrl_state == MANUAL_OPERATION ||
        robot->ctrl_state == STARTUP_STATE) {
      return false; // 切换模式退出函数
    }
    // 初始化视觉检测变量
    visual_t *ball_tmp = NULL;       // 视觉反馈下发的所有数据集合
    visual_t *find_ball = NULL;      // 最后符合条件的数据集合
    uint32_t jl = 9999999;           // 定义一个距离变量为无限大即可
    for (uint8_t i = 0; i < 14; i++) // for循环遍历14次
    {
      delay(1);                      // 延时1ms
      ball_tmp = visual::getBall(i); // 获取视觉下发数据
      if (pd == 0) { // 判断是color颜色还是除color颜色以外（0是color颜色）
        if (color == ball_tmp->type) // 判断寻找颜色是否相符
        {
          dt = (millis() -
                ball_tmp->timestamp); // 计算现在时间与数据获取时间的插值
          if (dt < 100)               // 判断是否小与100ms（最新消息）
          {
            // 定义一个x的向量差值xx
            int16_t xx;
            if (color == 1 || color == 2) { // 判断color是否为安全区
              xx = 160 -
                   (ball_tmp->x +
                    ball_tmp->w / 2); // 计算环面中心与视觉识别安全区的向量差
              if (abs(xx) < 12) {     // 判断向量差的绝对值是否小于12
                return true;          // 返回 true
              }
            }
          }
        }
      }
    }
    delay(10); // 延时10ms
  }
  return false; // 返回 false
}

/**
 * @brief 放球姿态控制
 通过调整放球姿态
 使小车始终90度放球
 <无需理会直接使用此函数即可>
 */
float last_angle1; // 摆放前角度
void corner_angle() {
  bool pd1;
  if (fabs((robot_->current_pos.yaw) + 90) <
      75) {                                // 判断目前角度是否满足90度摆放
    last_angle1 = robot_->current_pos.yaw; // 获取现在陀螺仪角度
    robot.imu_hold = true;                 // 陀螺仪保持
    robot.chassis_mode = POSTION_MODE;     // 小车进入里程计模式
    robot_->target_pos.x = robot_->current_pos.x;  // x目标位置为现在位置
    robot_->target_pos.y = robot_->current_pos.y;  // y目标位置为现在位置
    robot_->target_pos.yaw = LIMIT_ANGLE_180(-90); // 车身旋转至-90
    while (robot_->ctrl_state == AUTO_OPERATION)   // 在自动模式下
    {
      if (fabs(robot.target_pos.yaw - robot.current_pos.yaw) <
          1) // 目标陀螺仪角度与现在陀螺仪角度插值是否小于 1
      {
        break; // 退出循环
      }
      delay(10); // 延时10ms
    }
    delay(250);                       // 延时250ms
    pd1 = certain_home(&robot, c, 0); // 判断目前视觉模块正前方是否为本方安全区
    if (pd1) {                        // 是
      SetSpeed(0.3, 0, 0, 1.25);      // 向前移动一段时间
    } else if (!pd1) {                // 否
      if (last_angle1 < -90.0)        // 判断旋转前角度是否大于 -90度
      {
        MovePosition(robot_->current_pos.x, robot_->current_pos.y,
                     0);            // 小车姿态保持0度
        SetSpeed(-0.3, 0, 0, 0.75); // 向后倒退一段时间
        MovePosition(robot_->current_pos.x, robot_->current_pos.y,
                     -90);         // 小车姿态保持-90度
        SetSpeed(0.3, 0, 0, 1.25); // 向前移动一段时间
      }
      if (last_angle1 > -90.0) // 判断旋转前角度是否小于 -90度
      {
        MovePosition(robot_->current_pos.x, robot_->current_pos.y,
                     0);           // 小车姿态保持0度
        SetSpeed(0.3, 0, 0, 0.75); // 向前移动一段时间
        MovePosition(robot_->current_pos.x, robot_->current_pos.y,
                     -90);         // 小车姿态保持-90度
        SetSpeed(0.3, 0, 0, 1.25); // 向前移动一段时间
      }
    }
  } else if (fabs((robot_->current_pos.yaw)) <
             60) {                     // 判断车的姿态绝对值是为-60-60度
    robot.imu_hold = true;             // 陀螺仪保持
    robot.chassis_mode = POSTION_MODE; // 小车进入里程计模式
    robot_->target_pos.x = robot_->current_pos.x; // x目标位置为现在位置
    robot_->target_pos.y = robot_->current_pos.y; // y目标位置为现在位置
    robot_->target_pos.yaw = LIMIT_ANGLE_180(0);  // 车身旋转至0
    delay(700);                                   // 延时700ms
    SetSpeed(0.3, 0, 0, 1.25);                    // 向前开一段距离
  } else if (fabs(fabs(robot_->current_pos.yaw) - 180) <
             60) {                     // 判断车的姿态绝对值是否为120-240度
    robot.imu_hold = true;             // 陀螺仪保持
    robot.chassis_mode = POSTION_MODE; // 小车进入里程计模式
    robot_->target_pos.x = robot_->current_pos.x;  // x目标位置为现在位置
    robot_->target_pos.y = robot_->current_pos.y;  // y目标位置为现在位置
    robot_->target_pos.yaw = LIMIT_ANGLE_180(180); // 车身旋转至180
    delay(700);                                    // 延时700ms
    SetSpeed(0.3, 0, 0, 1.25);                     // 向前开一段距离
  }
}
/**
 * @brief 回家路径
 通过 Turn_angle 和 Turn_angle2
 两个转向函数以及速度控制使小车
 直接通过里程计开到本方安全区
 <- 无需理会直接使用此函数即可 ->
 */

float time2 = 0;
void go_home() {
  Turn_angle2();                     // 通过Turn_angle2函数转向中间偏位置
  robot.imu_hold = true;             // 陀螺仪保持
  robot_->chassis_mode = SPEED_MODE; // 改速度模式
  robot_->set_vel.linear_x = 0.4;    // x速度给0.4（转完向前跑）
  robot_->set_vel.linear_y = 0;      // y速度给0
  robot_->set_vel.angular_z = 0;     // z速度给0
  while (robot_->ctrl_state == AUTO_OPERATION) { // 自动模式下
    // 计算目标位置和实际位置插值
    float distance =
        sqrt(pow(robot_->current_pos.x - (home_xx + (home_x - 1.05)), 2) +
             pow(robot_->current_pos.y - (home_yy + (home_y + 0.95)), 2));
    ++time2;
    if (distance < 0.4 or
        time2 >
            250) { // 判断目标位置和实际位置差值是否小于0.4m 或 运行时间超出2.5s
      break;       // 退出循环
    }
    delay(10); // 延时10ms
  }
  time2 = 0;
  Turn_angle();                      // 通过Turn_angle函数转向本方安全区
  robot.imu_hold = true;             // 陀螺仪保持
  robot_->chassis_mode = SPEED_MODE; // 改速度模式
  robot_->set_vel.linear_x = 0.55;   // x速度给0.55（转完向前跑）
  robot_->set_vel.linear_y = 0;      // y速度给0
  robot_->set_vel.angular_z = 0;     // z速度给0
  while (robot_->ctrl_state == AUTO_OPERATION) { // 自动模式下
    // 计算目标位置和实际位置插值
    float distance = sqrt(pow(robot_->current_pos.x - home_x, 2) +
                          pow(robot_->current_pos.y - home_y, 2));
    if (distance < 0.4) { // 判断目标位置和实际位置差值是否小于0.4m
      break;              // 退出循环
    }
    delay(10); // 延时10ms
  }
}
/**
* @brief 单个物料检测（并不是每个机构都适用 不建议使用）
 通过读取舵机现在位置和只要一个
 物料时去做比较，做些动作使机构
 内只剩一个物料
<- 不建议使用 ->
 */
void only_one() {
  while (robot_->ctrl_state == AUTO_OPERATION) {
    if (robot.servo[0]->getAngle() > -57) {
      SetSpeed(0, 0, 2, 0.35); // 向前冲
      delay(50);
      robot.servo[0]->setAngle(-45); // 爪子张开
      delay(100);
      SetSpeed(-0.07, 0, 0, 0.25);   // 向前冲
      robot.servo[0]->setAngle(-62); // 爪子闭合
      delay(500);
    } else {
      break;
    }
    delay(50);
  }
  return;
}
/**
 * @brief 找求函数
 通过速度模式控制小车前后左右移动
 目的让小球坐落在小车视觉模块指定
 的坐标范围以内
 * @param robot 小车参数
 * @param color 寻找的颜色
 * @param pd 0仅仅此颜色 1除了此颜色其他颜色
 * @retuen true 找到并小球坐落在小车视觉模块指定的坐标范围以内
 * @retuen false 未找到 或 寻找时间超时
 */
bool VisualFindBall(Robot_t *robot, uint8_t color, uint8_t pd) {
  // 定义三个存储时间的变量
  uint32_t dt = 0;                  // 数据更新时间
  uint32_t ft = millis();           // 距离上一次找到的时间
  uint32_t ct = millis();           // 进入函数初始时间
  robot->imu_hold = false;          // 陀螺仪保持关闭
  robot->chassis_mode = SPEED_MODE; // 小车转换为速度模式
  while (1) {
    if ((millis() - ct) > 15000) // 判断超时时间是否大于15秒
    {
      return false; // 返回false
    }
    if (robot->ctrl_state == MANUAL_OPERATION ||
        robot->ctrl_state == STARTUP_STATE) {
      return false; // 切换模式自动退出
    }
    // 初始化视觉检测变量
    visual_t *ball_tmp = NULL;       // 视觉反馈下发的所有数据集合
    visual_t *find_ball = NULL;      // 最后符合条件的数据集合
    uint32_t jl = 9999999;           // 定义一个距离变量为无限大即可
    for (uint8_t i = 0; i < 14; i++) // for循环遍历14次
    {
      ball_tmp = visual::getBall(i); // 获取视觉下发数据
      if (pd == 0) { // 判断是color颜色还是除color颜色以外（0是color颜色）
        if (color == ball_tmp->type) // 判断寻找颜色是否相符
        {
          dt = (millis() -
                ball_tmp->timestamp); // 计算现在时间与数据获取时间的插值
          if (dt < 100)               // 判断是否小与100ms（最新消息）
          {
            // 定义两个常量为初步X,Y的向量差
            int16_t xx;
            int16_t yy;
            if (color == 1 || color == 2) {               // 判断是否为安全区
              xx = 160 - (ball_tmp->x + ball_tmp->w / 2); // 不可调
            } else {
              xx = 160 -
                   (ball_tmp->x +
                    ball_tmp->w / 2); // 320*320  理想情况是160装歪可能需要略调
            }
            yy = 160 -
                 (ball_tmp->y +
                  ball_tmp->h / 2); // 320*320  理想情况是160装歪可能需要略调
            if (((xx * xx) + (yy * yy)) < jl) // 计算距离的是否小于jl
            {
              find_ball = ball_tmp;       // 更新找到球的数据集合
              jl = (xx * xx) + (yy * yy); // 刷新最小距离数据
            }
          }
        }
      } else if (
          pd == 1) { // 判断是color颜色还是除color颜色以外（1为除color颜色以外）
        if (ball_tmp->type != 1 && ball_tmp->type != 2) { // 先把安全区数据过滤
          if (color != ball_tmp->type && color != 1 &&
              color != 2) // 判断视觉数据是否与color不同
          {
            dt = (millis() -
                  ball_tmp->timestamp); // 计算现在时间与数据获取时间的插值
            if (dt < 100)               // 判断是否小与100ms（最新消息）
            {
              // 定义两个常量为初步X,Y的向量差 xx,yy
              int16_t xx;
              int16_t yy;
              xx = 160 -
                   (ball_tmp->x +
                    ball_tmp->w / 2); // 320*320  理想情况是160装歪可能需要略调
              yy = 160 -
                   (ball_tmp->y +
                    ball_tmp->h / 2); // 320*320  理想情况是160装歪可能需要略调
              if (((xx * xx) + (yy * yy)) < jl) // 计算距离的是否小于jl
              {
                find_ball = ball_tmp;       // 更新找到球的数据集合
                jl = (xx * xx) + (yy * yy); // 刷新最小距离数据
              }
            }
          }
        }
      }
    }

    if (find_ball != NULL) // 判断是否获取到符合条件的数据集合
    {
      // 定义两个常量为X,Y的向量差
      int16_t x;
      int16_t y;
      ft = millis();                                 // 更新上一次找球时间
      if (color == 1 or color == 2) {                // 判断是否为安全区
        x = 160 - (find_ball->x + find_ball->w / 2); // 不可调
      } else {
        x = 160 - (find_ball->x +
                   find_ball->w / 2); // 320*320  理想情况是160装歪可能需要略调
      }
      y = 160 - (find_ball->y +
                 find_ball->h / 2); // 320*320  理想情况是160装歪可能需要略调
      robot->set_vel.linear_x = y * 0.0065; // 跟随太慢改比例系数 反运动加正负号
      robot->set_vel.linear_y = 0;          // 不适用横向平移量
      robot->set_vel.angular_z =
          x * 0.0075; // 跟随太慢改比例系数 反运动加正负号

      if (abs(x) <= 5 && abs(y) < 4) // 判断X,Y的向量差是否满足区间
      {
        robot->imu_hold = true; // 陀螺仪保持
        robot->target_pos.yaw =
            LIMIT_ANGLE_180(robot->current_pos.yaw); // 保持目前角度
        return true;                                 // 返回 true
      }
    } else {
      if ((millis() - ft) > 250) {       // 判断距离上次找球是否大于250ms
        robot->set_vel.linear_x = 0;     // x速度给0
        robot->set_vel.linear_y = 0;     // y速度给0
        robot->set_vel.angular_z = 0.85; // z速度给0.85 (旋转找球)
        if ((millis() - ft) > 10000)     // 判断距离上一次找球大于10s
        {
          return false; // 返回 false
        }
      } else {
        robot->set_vel.linear_x = 0;  // x速度给0
        robot->set_vel.linear_y = 0;  // y速度给0
        robot->set_vel.angular_z = 0; // z速度给0 （原地等待一会儿）
      }
    }
    delay(10); // 延时10ms
  }
}
/**
 * @brief 找球逻辑part1（案例）
 通过综合部分函数，使小车在
 五个点位进行寻找物料并把物
 料移至本方安全区内
 */
void PART1() {
  if (robot_->team_color == RobotColor::BLUE) {
    a = 3;
    b = 0;
    c = 1;
  } // 蓝方（通过读取拨码快速切换）
  else {
    a = 0;
    b = 3;
    c = 2;
  } // 红方（通过读取拨码快速切换）
  while (robot_->ctrl_state == AUTO_OPERATION) {
    while (robot_->ctrl_state == AUTO_OPERATION) {
      // if (notfind !=0){
      // switch (notfind)//通过switch的值判断到那个找球点位
      // {
      // 	case 1:
      // 		find1();//移动到点位1
      // 		break;
      // 	case 2:
      // 		find2();//移动到点位2
      // 		break;
      // 	case 3:
      // 		find3();//移动到点位3
      // 		break;
      // 	case 4:
      // 		find4();//移动到点位4
      // 		break;
      // }
      // }
      if (mine < 1) {
        pd = VisualFindBall(
            &robot, a, 0); // 0,0只找红球 1,0只找蓝色安全区 2,0只找红色安全区
                           // 3,0只找蓝色球 4,0只找黄球 5,0只找黑球
                           // 0,1除了红球以外的球 3,1除了蓝球以外的球
      } else {
        pd = VisualFindBall(
            &robot, b, 1); // 0,0只找红球 1,0只找蓝色安全区 2,0只找红色安全区
                           // 3,0只找蓝色球 4,0只找黄球 5,0只找黑球
                           // 0,1除了红球以外的球 3,1除了蓝球以外的球
      }
      if (pd) {
        robot.imu_hold = true; // 陀螺仪保持
        // SetSpeed(0.3, 0, 0,0.35);//向前冲一段
        // delay(20);
        robot.servo[0]->setAngle(30);  // 爪子张开（框抬起）的角度
        robot.servo[1]->setAngle(-30); // 爪子张开（框抬起）的角度
        // delay(20);// 延时20ms
        SetSpeed(0.6, 0, 0, 0.47); // 向前冲一段
        delay(200);// 延时20ms
        robot.servo[0]->setAngle(-16); // 爪子合拢（框放下）的角度
        robot.servo[1]->setAngle(13);  // 爪子合拢（框放下）的角度
        delay(300);// 延时300ms
        SetSpeed(-0.6, 0, 0, 0.3);
        mine = mine + 1; // mine次数叠加(找到的次数叠加)
        break;
      }
      if (!pd && robot_->ctrl_state == AUTO_OPERATION) {
        notfind = notfind + 1; // notfind次数叠加(未找到的次数叠加)
      }
    }
    go_home(); // 通过go_home函数用里程计走到安全区附近
    while (robot_->ctrl_state == AUTO_OPERATION) {
      pd = VisualFindBall(
          &robot, c,
          0); // 0,0只找红球 1,0只找蓝色安全区 2,0只找红色安全区 3,0只找蓝色球
              // 4,0只找黄球 5,0只找黑球 0,1除了红球以外的球 3,1除了蓝球以外的球
      if (pd) {
        robot.imu_hold = true; // 陀螺仪保持
        // delay(300);// 延时300ms
        corner_angle(); // 调整车身姿态
        // delay(300);// 延时300ms
        robot.servo[0]->setAngle(30);
        robot.servo[1]->setAngle(-30);
        // delay(500);// 延时500ms
        home_x = robot_->current_pos.x; // 安全区的里程计相对坐标X刷新
        home_y = robot_->current_pos.y; // 安全区的里程计相对坐标Y刷新
        break;
      } else {
        Turn_angle();               // 转向里程计的家的方向
        robot.imu_hold = true;      // 陀螺仪保持
        SetSpeed(0.25, 0, 0, 0.75); // 向前冲一段
        delay(300);                 // 延时300ms
      }
    }
    robot.imu_hold = false;        // 陀螺仪保持
    SetSpeed(-0.5, 0, 0, 0.5);     // 后退一段距离
    SetSpeed(0, 0, 0.8, 1.6);      // 后退一段距离
    robot.servo[0]->setAngle(-16); // 爪子合拢（框放下）的角度
    robot.servo[1]->setAngle(13);  // 爪子合拢（框放下）的角度
  }
  return; // 返回
}
void PART2() {
  if (io::getIN1()) {
    a = 3;
    b = 0;
    c = 1;
  } // 红方（通过读取拨码快速切换）
  else {
    a = 0;
    b = 3;
    c = 2;
  } // 蓝方（通过读取拨码快速切换）
  while (robot_->ctrl_state == AUTO_OPERATION) {
    while (robot_->ctrl_state == AUTO_OPERATION) {
      if (notfind != 0) {
        switch (notfind) // 通过switch的值判断到那个找球点位
        {
        case 1:
          find1(); // 移动到点位1
          break;
        case 2:
          find2(); // 移动到点位2
          break;
        case 3:
          find3(); // 移动到点位3
          break;
        case 4:
          find4(); // 移动到点位4
          break;
        }
      }
      if (mine < 2) {
        pd = VisualFindBall(
            &robot, a, 0); // 0,0只找红球 1,0只找蓝色安全区 2,0只找红色安全区
                           // 3,0只找蓝色球 4,0只找黄球 5,0只找黑球
                           // 0,1除了红球以外的球 3,1除了蓝球以外的球
      } else {
        pd = VisualFindBall(
            &robot, b, 1); // 0,0只找红球 1,0只找蓝色安全区 2,0只找红色安全区
                           // 3,0只找蓝色球 4,0只找黄球 5,0只找黑球
                           // 0,1除了红球以外的球 3,1除了蓝球以外的球
      }
      if (pd) {
        robot.imu_hold = true;         // 陀螺仪保持
        robot.servo[2]->setAngle(125); // 云台舵机向下（低头）
        SetSpeed(0.3, 0, 0, 0.35);     // 向前冲一段
        delay(20);
        robot.servo[0]->setAngle(-35);    // 爪子张开（框抬起）的角度
        robot.servo[1]->setAngle(100);    // 爪子张开（框抬起）的角度
        delay(20);                        // 延时20ms
        SetSpeed(0.35, 0, 0, 0.35);       // 向前冲一段
        delay(20);                        // 延时20ms
        robot.servo[0]->setAngle(-60);    // 爪子合拢（框放下）的角度
        robot.servo[1]->setAngle(124);    // 爪子合拢（框放下）的角度
        delay(500);                       // 延时300ms
        pd2 = certain_ball(&robot, b, 1); // 检测里面的球是否不为对方颜色
        if (pd2 == 1) {
          mine = mine + 1;              // mine次数叠加(找到的次数叠加)
          robot.servo[2]->setAngle(97); // 云台舵机向上（抬头）
          delay(100);                   // 延时100ms
          break;
        } else if (pd2 == 2) {
          robot.servo[0]->setAngle(-35); // 爪子张开（框抬起）的角度
          robot.servo[1]->setAngle(100); // 爪子张开（框抬起）的角度
          SetSpeed(-0.25, 0, 0, 0.4);    // 向后移一段
          robot.servo[0]->setAngle(-60); // 爪子合拢（框放下）的角度
          robot.servo[1]->setAngle(124); // 爪子合拢（框放下）的角度
          robot.servo[2]->setAngle(97);  // 云台舵机向上（抬头）
          delay(100);                    // 延时100ms
        } else {
          robot.servo[2]->setAngle(97); // 云台舵机向上（抬头）
          delay(100);                   // 延时100ms
        }
      }
      if (!pd && robot_->ctrl_state == AUTO_OPERATION) {
        notfind = notfind + 1; // notfind次数叠加(未找到的次数叠加)
      }
    }
    go_home(); // 通过go_home函数用里程计走到安全区附近
    while (robot_->ctrl_state == AUTO_OPERATION) {
      pd = VisualFindBall(
          &robot, c,
          0); // 0,0只找红球 1,0只找蓝色安全区 2,0只找红色安全区 3,0只找蓝色球
              // 4,0只找黄球 5,0只找黑球 0,1除了红球以外的球 3,1除了蓝球以外的球
      if (pd) {
        robot.imu_hold = true;          // 陀螺仪保持
        delay(300);                     // 延时300ms
        corner_angle();                 // 调整车身姿态
        delay(300);                     // 延时300ms
        robot.servo[0]->setAngle(-40);  // 爪子张开（框抬起）的角度
        robot.servo[1]->setAngle(104);  // 爪子张开（框抬起）的角度
        delay(500);                     // 延时500ms
        home_x = robot_->current_pos.x; // 安全区的里程计相对坐标X刷新
        home_y = robot_->current_pos.y; // 安全区的里程计相对坐标Y刷新
        break;
      } else {
        Turn_angle();               // 转向里程计的家的方向
        robot.imu_hold = true;      // 陀螺仪保持
        SetSpeed(0.25, 0, 0, 0.75); // 向前冲一段
        delay(300);                 // 延时300ms
      }
    }
    robot.imu_hold = false;        // 陀螺仪保持
    SetSpeed(-0.5, 0, 0, 0.5);     // 后退一段距离
    SetSpeed(0, 0, 0.8, 1.6);      // 后退一段距离
    robot.servo[0]->setAngle(-60); // 爪子合拢（框放下）的角度
    robot.servo[1]->setAngle(124); // 爪子合拢（框放下）的角度
  }
  return; // 返回
}

void find1() {
  // 将机器人移动到当前位置，面朝0度
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 0);
  // 移动机器人到计算后的家位置，仍然面朝0度
  MovePosition(0.4 + (home_x - 1.05), (home_y + 0.95), 0);
  delay(100); // 等待100毫秒，以确保机器人到达目标位置
  // 将机器人转动到90度面朝
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 90);
  // 移动机器人到新的计算位置，面朝90度
  MovePosition(0.4 + (home_x - 1.05), 0.5 + (home_y + 0.95), 90);
  delay(300); // 等待300毫秒，以确保机器人到达目标位置
}

void find2() {
  // 将机器人移动到当前位置，面朝0度
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 0);
  // 移动机器人到计算后的家位置，仍然面朝0度
  MovePosition(0.4 + (home_x - 1.05), (home_y + 0.95),
               0); // 到达目标位置(1.05, 0, 0)
  delay(100);      // 等待100毫秒，以确保机器人到达目标位置
  // 将机器人转动到90度面朝
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 90);
  // 移动机器人到新的计算位置，面朝90度
  MovePosition(0.4 + (home_x - 1.05), -0.5 + (home_y + 0.95),
               90); // 到达目标位置(1.05, 0, 90)
  delay(300);       // 等待300毫秒，以确保机器人到达目标位置
}

void find3() {
  // 将机器人移动到当前位置，面朝0度
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 0);
  // 移动机器人到计算后的家位置，仍然面朝0度
  MovePosition(1.8 + (home_x - 1.05), (home_y + 0.95),
               0); // 到达目标位置(1.05, 0, 0)
  delay(100);      // 等待100毫秒，以确保机器人到达目标位置
  // 将机器人转动到90度面朝
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 90);
  // 移动机器人到新的计算位置，面朝90度
  MovePosition(1.8 + (home_x - 1.05), -0.5 + (home_y + 0.95),
               90); // 到达目标位置(1.05, 0, 90)
  delay(300);       // 等待300毫秒，以确保机器人到达目标位置
}

void find4() {
  // 将机器人移动到当前位置，面朝0度
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 0);
  // 移动机器人到计算后的家位置，仍然面朝0度
  MovePosition(1.8 + (home_x - 1.05), (home_y + 0.95),
               0); // 到达目标位置(1.05, 0, 0)
  delay(100);      // 等待100毫秒，以确保机器人到达目标位置
  // 将机器人转动到90度面朝
  MovePosition(robot_->current_pos.x, robot_->current_pos.y, 90);
  // 移动机器人到新的计算位置，面朝90度
  MovePosition(1.8 + (home_x - 1.05), 0.5 + (home_y + 0.95),
               90); // 到达目标位置(1.05, 0, 90)
  delay(300);       // 等待300毫秒，以确保机器人到达目标位置
}

} // namespace auto_ctrl
