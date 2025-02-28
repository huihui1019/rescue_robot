#include "gui_main.h"
#include "keypad.h"
#include "robot.h"

static lv_obj_t *parent_root;
static lv_timer_t *_timer;
static lv_obj_t *ui_robot_state_panel;
static lv_obj_t *ui_robot_state_label;
static lv_obj_t *ui_num_panel;
static lv_obj_t *ui_num_label;
static lv_obj_t *ui_x_label;
static lv_obj_t *ui_y_label;
static lv_obj_t *ui_yaw_label;
static lv_obj_t *ui_textarea;
static lv_obj_t *ui_battery_label;

const char *battery_anim[5] = {LV_SYMBOL_BATTERY_EMPTY, LV_SYMBOL_BATTERY_1,
                               LV_SYMBOL_BATTERY_2, LV_SYMBOL_BATTERY_3,
                               LV_SYMBOL_BATTERY_FULL};
// 定义电压阈值数组，与battery_anim一一对应
float voltage_thresholds[] = {0, 10.5, 11, 11.5, 12};

LV_FONT_DECLARE(ui_font_dingtalk);

static void event_key_handler(lv_event_t *e) {
  /* last_time用来记录两次按键的间隔时间，实现双击切换界面，防止误触 */
  static uint32_t last_time = 0;

  if (e->code == LV_EVENT_KEY) {
    const uint32_t key = lv_indev_get_key(lv_indev_get_act());

    if (key == KEY2_PRESS_DOWN) {
      if (millis() - last_time < 500) {
        robot.ctrl_state = DEBUG_STATE; // 状态修改，进入调试状态
        page_fast_load(PAGE_IO, false);
      }
      last_time = millis();
    } else if (key == KEY3_PRESS_DOWN) {
      if (millis() - last_time < 500) {
        chassis->resetOdom();
        imu::reset();
      }
      last_time = millis();
    } else if (key == KEY4_PRESS_DOWN) {
      robot.ctrl_state = STARTUP_STATE;
      robot.chassis_mode = SAFE_MODE;
      robot.motor[0]->clearStatus();
      robot.motor[1]->clearStatus();
      robot.motor[2]->clearStatus();
      robot.motor[3]->clearStatus();
      io::setSG4(1, 0);
    } else if (key == KEY1_PRESS_DOWN) {
      robot.auto_path = robot.auto_path + 1;
      if (robot.auto_path > 4) {
        robot.auto_path = 1;
      }
    } else if (key == KEY5_PRESS_DOWN) {
      robot.ctrl_state = AUTO_OPERATION;
      delay(300);
      robot.auto_sub_state = START;
    }
  }
}

static void timer_handler(lv_timer_t *timer) {
  static int32_t tick[2] = {0};

  if (millis() - tick[0] > 100) {
    tick[0] = millis();
    switch (robot.ctrl_state) {
    case STARTUP_STATE:
      lv_label_set_text(ui_robot_state_label, "S");
      lv_label_set_text_fmt(ui_num_label, "%d", robot.auto_path);
      break;
    case MANUAL_OPERATION:
      lv_label_set_text(ui_robot_state_label, "M");
      lv_label_set_text_fmt(ui_num_label, "%d", robot.auto_path);
      break;
    case AUTO_OPERATION:
      lv_label_set_text(ui_robot_state_label, "A");
      lv_label_set_text_fmt(ui_num_label, "%d", robot.auto_path);
      break;
    case DEBUG_STATE:
      lv_label_set_text(ui_robot_state_label, "D");
      break;
    default:
      break;
    }

    lv_label_set_text_fmt(ui_x_label, "X: %.2f(m)", robot.current_pos.x);
    lv_label_set_text_fmt(ui_y_label, "Y: %.2f(m)", robot.current_pos.y);
    lv_label_set_text_fmt(ui_yaw_label, "Yaw: %.2f(deg)",
                          robot.current_pos.yaw);
  }

  if (millis() - tick[1] > 1000) {
    tick[1] = millis();
    int battery_index;
    for (battery_index = 4;
         battery_index > 0 &&
         robot.battery_voltage < voltage_thresholds[battery_index];
         --battery_index) {
    }
    if (robot.battery_voltage == 0) {
      battery_index = 0;
    }
    lv_label_set_text_fmt(ui_battery_label, "%.1fv %s", robot.battery_voltage,
                          battery_anim[battery_index]);
  }
}

void gui_main_init(lv_obj_t *root) {
  parent_root = root;
  lv_obj_clear_flag(parent_root, LV_OBJ_FLAG_SCROLLABLE);
  lv_group_add_obj(lv_group_get_default(), parent_root);
  lv_obj_add_event_cb(parent_root, event_key_handler, LV_EVENT_KEY, NULL);

  ui_robot_state_panel = lv_obj_create(parent_root);
  lv_obj_set_width(ui_robot_state_panel, 100);
  lv_obj_set_height(ui_robot_state_panel, 90);
  lv_obj_set_x(ui_robot_state_panel, -60);
  lv_obj_set_y(ui_robot_state_panel, -55);
  lv_obj_set_align(ui_robot_state_panel, LV_ALIGN_CENTER);
  lv_obj_clear_flag(ui_robot_state_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(ui_robot_state_panel, lv_color_hex(0xC0FF6B),
                            LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_robot_state_panel, 255,
                          LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_color(ui_robot_state_panel, lv_color_hex(0xD5D5D5),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_opa(ui_robot_state_panel, 255,
                              LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_robot_state_label = lv_label_create(ui_robot_state_panel);
  lv_obj_set_align(ui_robot_state_label, LV_ALIGN_CENTER);
  lv_label_set_text(ui_robot_state_label, "S");
  lv_obj_set_style_text_font(ui_robot_state_label, &lv_font_montserrat_48,
                             LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_num_panel = lv_obj_create(parent_root);
  lv_obj_set_width(ui_num_panel, 100);
  lv_obj_set_height(ui_num_panel, 90);
  lv_obj_set_x(ui_num_panel, 60);
  lv_obj_set_y(ui_num_panel, -55);
  lv_obj_set_align(ui_num_panel, LV_ALIGN_CENTER);
  lv_obj_clear_flag(ui_num_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(ui_num_panel, lv_color_hex(0x638FF4),
                            LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_num_panel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_color(ui_num_panel, lv_color_hex(0xD5D5D5),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_opa(ui_num_panel, 255,
                              LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_num_label = lv_label_create(ui_num_panel);
  lv_obj_set_align(ui_num_label, LV_ALIGN_CENTER);
  lv_label_set_text(ui_num_label, "0");
  lv_obj_set_style_text_font(ui_num_label, &lv_font_montserrat_48,
                             LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_x_label = lv_label_create(parent_root);
  lv_obj_set_x(ui_x_label, 10);
  lv_obj_set_y(ui_x_label, 15);
  lv_obj_set_align(ui_x_label, LV_ALIGN_LEFT_MID);
  lv_label_set_text(ui_x_label, "X:123.45(m)");

  ui_y_label = lv_label_create(parent_root);
  lv_obj_set_x(ui_y_label, 10);
  lv_obj_set_y(ui_y_label, 55);
  lv_obj_set_align(ui_y_label, LV_ALIGN_LEFT_MID);
  lv_label_set_text(ui_y_label, "Y:123.45(m)");

  ui_yaw_label = lv_label_create(parent_root);
  lv_obj_set_x(ui_yaw_label, 10);
  lv_obj_set_y(ui_yaw_label, 95);
  lv_obj_set_align(ui_yaw_label, LV_ALIGN_LEFT_MID);
  lv_label_set_text(ui_yaw_label, "Yaw:123.45(deg)");

  // 电池标志
  //  battery_anim_frame = 0;
  ui_battery_label = lv_label_create(parent_root);
  lv_obj_set_width(ui_battery_label, LV_SIZE_CONTENT);
  lv_obj_set_height(ui_battery_label, LV_SIZE_CONTENT);
  lv_obj_set_x(ui_battery_label, -5);
  lv_obj_set_y(ui_battery_label, 5);
  lv_obj_set_align(ui_battery_label, LV_ALIGN_TOP_RIGHT);
  lv_label_set_text_fmt(ui_battery_label, "%.1fv %s", 12.0, battery_anim[0]);

  ui_textarea = lv_textarea_create(parent_root);
  lv_obj_set_width(ui_textarea, 80);
  lv_obj_set_height(ui_textarea, 110);
  lv_obj_set_x(ui_textarea, 70);
  lv_obj_set_y(ui_textarea, 50);
  lv_obj_set_align(ui_textarea, LV_ALIGN_CENTER);
  lv_textarea_set_text(ui_textarea, "按键3重置里程计");
  lv_obj_set_style_text_font(ui_textarea, &ui_font_dingtalk,
                             LV_PART_MAIN | LV_STATE_DEFAULT);

  _timer = lv_timer_create(timer_handler, 100, NULL);
}

void gui_main_focus(void) {
  robot.ctrl_state = STARTUP_STATE;
  lv_group_focus_obj(parent_root);
  lv_timer_resume(_timer);
}

void gui_main_del(void) { lv_timer_pause(_timer); }
