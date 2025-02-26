#include "gui_imu.h"
#include "robot.h"
#include "keypad.h"

LV_FONT_DECLARE(ui_font_dingtalk);

static lv_obj_t *parent_root;
static lv_obj_t *ui_gx;
static lv_obj_t *ui_gy;
static lv_obj_t *ui_gz;
static lv_obj_t *ui_ax;
static lv_obj_t *ui_ay;
static lv_obj_t *ui_az;
static lv_obj_t *ui_pitch;
static lv_obj_t *ui_yaw;
static lv_obj_t *ui_roll;
static lv_obj_t *ui_hint_label;
static lv_timer_t *_timer;

static void event_key_handler(lv_event_t *e)
{
    if (e->code == LV_EVENT_KEY)
    {
        const uint32_t key = lv_indev_get_key(lv_indev_get_act());

        if (key == KEY1_PRESS_DOWN)
        {
            page_fast_load(PAGE_IO, true);
        }
        else if (key == KEY2_PRESS_DOWN)
        {
            page_fast_load(PAGE_MOTOR, true);
        }
        else if (key == KEY3_PRESS_DOWN)
        {
            imu::startCalibration(0, NULL);
        }
        
    }
}

static void timer_handler(lv_timer_t * timer)
{
    double gx, gy, gz;
    double ax, ay, az;
    double roll, pitch, yaw;

    if (imu::isCalibration())
    {
        lv_label_set_text(ui_hint_label, "陀螺仪校准中...");
        lv_label_set_text_fmt(ui_gx, "gx:%.2f", 0.0);
        lv_label_set_text_fmt(ui_gy, "gy:%.2f", 0.0);
        lv_label_set_text_fmt(ui_gz, "gz:%.2f", 0.0);

        lv_label_set_text_fmt(ui_ax, "ax:%.2f", 0.0);
        lv_label_set_text_fmt(ui_ay, "ay:%.2f", 0.0);
        lv_label_set_text_fmt(ui_az, "az:%.2f", 0.0);

        lv_label_set_text_fmt(ui_pitch, "pitch:%.2f", 0.0);
        lv_label_set_text_fmt(ui_yaw, "yaw:%.2f", 0.0);
        lv_label_set_text_fmt(ui_roll, "roll:%.2f", 0.0);
    }
    else
    {
        lv_label_set_text(ui_hint_label, "按键3校准陀螺仪");
        imu::getGyro(&gx, &gy, &gz);
        lv_label_set_text_fmt(ui_gx, "gx:%.2f", gx);
        lv_label_set_text_fmt(ui_gy, "gy:%.2f", gy);
        lv_label_set_text_fmt(ui_gz, "gz:%.2f", gz);

        imu::getAccle(&ax, &ay, &az);
        lv_label_set_text_fmt(ui_ax, "ax:%.2f", ax);
        lv_label_set_text_fmt(ui_ay, "ay:%.2f", ay);
        lv_label_set_text_fmt(ui_az, "az:%.2f", az);

        imu::getAngle(&roll, &pitch, &yaw);
        lv_label_set_text_fmt(ui_pitch, "pitch:%.2f", pitch);
        lv_label_set_text_fmt(ui_yaw, "yaw:%.2f", yaw);
        lv_label_set_text_fmt(ui_roll, "roll:%.2f", roll);
    }
}


void gui_imu_init(lv_obj_t* root)
{
    lv_obj_t *label;
    lv_obj_t *panel;

    parent_root = root;
    lv_group_add_obj(lv_group_get_default(), parent_root);
    lv_obj_add_event_cb(parent_root, event_key_handler, LV_EVENT_KEY, NULL);

    label = lv_label_create(parent_root);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 10);
    lv_obj_set_align(label, LV_ALIGN_TOP_MID);
    lv_label_set_text(label, "IMU调试界面");
    lv_obj_set_style_text_color(label, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);

    label = lv_label_create(parent_root);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, 10);
    lv_obj_set_y(label, 10);
    lv_obj_set_align(label, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(label, "<-K1");
    lv_obj_set_style_text_color(label, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    label = lv_label_create(parent_root);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, -10);
    lv_obj_set_y(label, 10);
    lv_obj_set_align(label, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(label, "K2->");
    lv_obj_set_style_text_color(label, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    panel = lv_obj_create(parent_root);
    lv_obj_set_width(panel, 100);
    lv_obj_set_height(panel, 110);
    lv_obj_set_x(panel, -60);
    lv_obj_set_y(panel, -35);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(panel, lv_color_hex(0xADEDC2), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_gx = lv_label_create(panel);
    lv_obj_set_x(ui_gx, 20);
    lv_obj_set_y(ui_gx, 0);
    lv_label_set_text(ui_gx, "gx:1.23");
    lv_obj_set_align(ui_gx, LV_ALIGN_LEFT_MID);

    ui_gy = lv_label_create(panel);
    lv_obj_set_x(ui_gy, 20);
    lv_obj_set_y(ui_gy, -70);
    lv_label_set_text(ui_gy, "gy:1.23");
    lv_obj_set_align(ui_gy, LV_ALIGN_LEFT_MID);

    ui_gz = lv_label_create(panel);
    lv_obj_set_x(ui_gz, 20);
    lv_obj_set_y(ui_gz, 50);
    lv_label_set_text(ui_gz, "gz:1.23");
    lv_obj_set_align(ui_gz, LV_ALIGN_LEFT_MID);

    panel = lv_obj_create(parent_root);
    lv_obj_set_width(panel, 100);
    lv_obj_set_height(panel, 110);
    lv_obj_set_x(panel, 60);
    lv_obj_set_y(panel, -35);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(panel, lv_color_hex(0xC4F4F9), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ax = lv_label_create(panel);
    lv_obj_set_x(ui_ax, 20);
    lv_obj_set_y(ui_ax, 0);
    lv_label_set_text(ui_ax, "ax:1.23");
    lv_obj_set_align(ui_ax, LV_ALIGN_LEFT_MID);

    ui_ay = lv_label_create(panel);
    lv_obj_set_x(ui_ay, 20);
    lv_obj_set_y(ui_ay, -70);
    lv_label_set_text(ui_ay, "ay:1.23");
    lv_obj_set_align(ui_ay, LV_ALIGN_LEFT_MID);

    ui_az = lv_label_create(panel);
    lv_obj_set_x(ui_az, 20);
    lv_obj_set_y(ui_az, 50);
    lv_label_set_text(ui_az, "az:1.23");
    lv_obj_set_align(ui_az, LV_ALIGN_LEFT_MID);

    panel = lv_obj_create(parent_root);
    lv_obj_set_width(panel, 220);
    lv_obj_set_height(panel, 59);
    lv_obj_set_x(panel, 0);
    lv_obj_set_y(panel, 54);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_SPACE_BETWEEN);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x64BFF4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_pitch = lv_label_create(panel);
    lv_label_set_text(ui_pitch, "pitch:1.23");
    lv_obj_set_align(ui_pitch, LV_ALIGN_CENTER);

    ui_yaw = lv_label_create(panel);
    lv_label_set_text(ui_yaw, "yaw:1.23");
    lv_obj_set_align(ui_yaw, LV_ALIGN_CENTER);

    ui_roll = lv_label_create(panel);
    lv_label_set_text(ui_roll, "roll:1.23");
    lv_obj_set_align(ui_roll, LV_ALIGN_CENTER);

    
    panel = lv_obj_create(parent_root);
    lv_obj_set_width(panel, 220);
    lv_obj_set_height(panel, 24);
    lv_obj_set_x(panel, 0);
    lv_obj_set_y(panel, 100);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0xE949DB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_hint_label = lv_label_create(panel);
    lv_obj_set_align(ui_hint_label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_hint_label, "按键3校准陀螺仪");
    lv_obj_set_style_text_font(ui_hint_label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);

    _timer = lv_timer_create(timer_handler, 100, NULL);
}

void gui_imu_focus()
{
    lv_group_focus_obj(parent_root);
}

void gui_imu_del(void)
{
    lv_timer_del(_timer);
}