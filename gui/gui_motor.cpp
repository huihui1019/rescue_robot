#include "gui_motor.h"
#include "keypad.h"
#include "robot.h"

#define KEY_DOWN(KEY_NAME) (key == KEY_NAME##_PRESS_DOWN)

LV_FONT_DECLARE(ui_font_dingtalk);

enum
{
    UI_PAGE_SELECT = -1,
    UI_PAGE_CONNECT_DEV = 0,
    UI_PAGE_MOTOR_SPEED = 1,
    UI_PAGE_MOTOR_POS = 2,
    UI_PAGE_SET_ID = 3,
    UI_PAGE_TOTAL_NUM,
};

static lv_obj_t *parent_root;
static lv_obj_t *ui_group_panel;
static lv_obj_t *ui_pages[UI_PAGE_TOTAL_NUM];
static lv_obj_t *ui_dev_id;
static lv_obj_t *ui_set_speed;
static lv_obj_t *ui_chart_speed;
static lv_chart_series_t *ui_chart_speed_series;
static lv_obj_t *ui_set_pos;
static lv_obj_t *ui_set_id;
static lv_obj_t *ui_set_id_tip;
static lv_timer_t *_timer;
static CFF::HubMotor *_motor = NULL;

static int16_t ui_ctrl_state = UI_PAGE_SELECT;
static int16_t now_page = 0;
static lv_obj_t *select_page;
static lv_obj_t *last_page;
static bool is_connect = false;
static int16_t speed_range[2]={-100, 100};

static void event_key_handler(lv_event_t *e)
{
    if (e->code == LV_EVENT_KEY)
    {
        const uint32_t key = lv_indev_get_key(lv_indev_get_act());
        if (ui_ctrl_state == UI_PAGE_SELECT) // 应用功能切换
        {
            if (KEY_DOWN(KEY1))
            {
                page_fast_load(PAGE_IMU, true);
                return;
            }
            else if (KEY_DOWN(KEY2))
            {
                page_fast_load(PAGE_SERVO, true);
                return;
            }

            if (KEY_DOWN(KEY5) && now_page<(UI_PAGE_TOTAL_NUM-1))
            {
                now_page++;
            }
            else if (KEY_DOWN(KEY4) && now_page>0)
            {
                now_page--;
            }
            else if (KEY_DOWN(KEY3))
            {
                ui_ctrl_state = now_page;
            }
        }
        else if (ui_ctrl_state == UI_PAGE_CONNECT_DEV)   // 连接设备
        {
            if (KEY_DOWN(KEY3))
            {
                if (_motor != NULL)
                {
                    if (_motor->getId() != lv_spinbox_get_value(ui_dev_id))
                    {
                        delete _motor;
                        _motor = NULL;
                        
                    }
                }
                if (_motor == NULL)
                {
                    _motor = motor::Register(lv_spinbox_get_value(ui_dev_id));
                }
                ui_ctrl_state = UI_PAGE_SELECT;
            }
            spinbox_controls(ui_dev_id, KEY_DOWN(KEY4), KEY_DOWN(KEY5), KEY_DOWN(KEY2), KEY_DOWN(KEY1));
        }
        else if (ui_ctrl_state == UI_PAGE_MOTOR_SPEED)  // 速度控制
        {
            if (KEY_DOWN(KEY3))
            {
                _motor->setSpeed(0);
                ui_ctrl_state = UI_PAGE_SELECT;
            }
            spinbox_controls(ui_set_speed, KEY_DOWN(KEY4), KEY_DOWN(KEY5), KEY_DOWN(KEY2), KEY_DOWN(KEY1));
        }
        else if (ui_ctrl_state == UI_PAGE_SET_ID)  // 修改ID
        {
            if (KEY_DOWN(KEY1) && _motor != nullptr)
            {
                if (_motor->setId(lv_spinbox_get_value(ui_set_id)))
                {
                    lv_label_set_text(ui_set_id_tip, "修改ID成功");
                }
                else
                {
                    lv_label_set_text(ui_set_id_tip, "修改ID失败");
                }
            }
            else if (KEY_DOWN(KEY3))
            {
                ui_ctrl_state = UI_PAGE_SELECT;
            }
            spinbox_controls(ui_set_id, KEY_DOWN(KEY4), KEY_DOWN(KEY5), false, false);
        }
        else if (ui_ctrl_state == UI_PAGE_MOTOR_POS)
        {
            if (KEY_DOWN(KEY3))
            {
                ui_ctrl_state = UI_PAGE_SELECT;
            }
            spinbox_controls(ui_set_pos, KEY_DOWN(KEY4), KEY_DOWN(KEY5), KEY_DOWN(KEY2), KEY_DOWN(KEY1));
            _motor->setPosition(HM_ANGLE_TO_VAL(lv_spinbox_get_value(ui_set_pos)));
        }

        select_page=ui_pages[now_page];
        pages_switching(select_page, last_page, ui_ctrl_state == UI_PAGE_SELECT);
        last_page = select_page;
    }

    
}

static void timer_handler(lv_timer_t * timer)
{
    int16_t set_speed, get_speed;
    if (_motor == NULL)
    {
        return;
    }

    switch (ui_ctrl_state)
    {
    case UI_PAGE_MOTOR_SPEED:
        _motor->setSpeed(lv_spinbox_get_value(ui_set_speed));
        get_speed = _motor->getSpeed();
        if (get_speed<speed_range[0])
        {
            speed_range[0] = get_speed;
            lv_chart_set_range(ui_chart_speed, LV_CHART_AXIS_PRIMARY_Y, speed_range[0], speed_range[1]);
        }
        if (get_speed>speed_range[1])
        {
            speed_range[1] = get_speed;
            lv_chart_set_range(ui_chart_speed, LV_CHART_AXIS_PRIMARY_Y, speed_range[0], speed_range[1]);
        }
        lv_chart_set_next_value(ui_chart_speed, ui_chart_speed_series, get_speed);
        break;
    default:
        break;
    }
}

void gui_motor_init(lv_obj_t* root)
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
    lv_label_set_text(label, "电机调试界面");
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

    ui_group_panel = lv_obj_create(parent_root);
    lv_obj_set_width(ui_group_panel, 240);
    lv_obj_set_height(ui_group_panel, 195);
    lv_obj_set_x(ui_group_panel, 0);
    lv_obj_set_y(ui_group_panel, 20);
    lv_obj_set_align(ui_group_panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_group_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_bg_color(ui_group_panel, lv_palette_darken(LV_PALETTE_GREY, 3), LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_pages[UI_PAGE_CONNECT_DEV] = create_page(ui_group_panel, "连接设备");
    ui_pages[UI_PAGE_MOTOR_SPEED] = create_page(ui_group_panel, "速度控制");
    ui_pages[UI_PAGE_MOTOR_POS] = create_page(ui_group_panel, "位置控制");
    ui_pages[UI_PAGE_SET_ID] = create_page(ui_group_panel, "修改ID");

    /* 设置id界面 */
    panel = lv_obj_create(ui_pages[UI_PAGE_CONNECT_DEV]);
    lv_obj_set_size(panel, LV_PCT(100), 50);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    label = lv_label_create(panel);
    lv_label_set_text(label, "设备ID");
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_dev_id = lv_spinbox_create(panel);
    lv_obj_set_width(ui_dev_id, 40);
    lv_obj_set_height(ui_dev_id, 40);
    lv_obj_set_align(ui_dev_id, LV_ALIGN_CENTER);
    lv_spinbox_set_digit_format(ui_dev_id, 1, 0);
    lv_spinbox_set_range(ui_dev_id, 1, 8);
    lv_spinbox_set_value(ui_dev_id, 1);
    lv_obj_set_style_bg_opa(ui_dev_id, LV_OPA_TRANSP, LV_PART_CURSOR | LV_STATE_DEFAULT);   // 隐藏光标
    lv_obj_set_style_text_opa(ui_dev_id, LV_OPA_TRANSP, LV_PART_CURSOR | LV_STATE_DEFAULT); // 隐藏光标

    /* 速度控制界面 */
    panel = lv_obj_create(ui_pages[UI_PAGE_MOTOR_SPEED]);
    lv_obj_set_size(panel, LV_PCT(100), 140);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    label = lv_label_create(panel);
    lv_obj_set_x(label, -50);
    lv_obj_set_y(label, -20);
    lv_label_set_text(label, "目标速度");
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_set_speed = lv_spinbox_create(panel);
    lv_obj_set_x(ui_set_speed, -50);
    lv_obj_set_y(ui_set_speed, 20);
    lv_obj_set_width(ui_set_speed, 80);
    lv_obj_set_height(ui_set_speed, 40);
    lv_obj_set_align(ui_set_speed, LV_ALIGN_CENTER);
    lv_spinbox_set_digit_format(ui_set_speed, 3, 0);
    lv_spinbox_set_range(ui_set_speed, -500, 500);
    lv_spinbox_set_value(ui_set_speed, 0);
    ui_chart_speed = lv_chart_create(panel);
    lv_obj_set_x(ui_chart_speed, 60);
    lv_obj_set_y(ui_chart_speed, 0);
    lv_obj_set_width(ui_chart_speed, 60);
    lv_obj_set_height(ui_chart_speed, 140);
    lv_obj_set_align(ui_chart_speed, LV_ALIGN_CENTER);
    lv_chart_set_type(ui_chart_speed, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(ui_chart_speed, 5);
    lv_chart_set_range(ui_chart_speed, LV_CHART_AXIS_PRIMARY_Y, speed_range[0], speed_range[1]);
    lv_chart_set_axis_tick(ui_chart_speed, LV_CHART_AXIS_PRIMARY_Y, 5, 5, 5, 2, true, 50);
    lv_obj_set_style_bg_color(ui_chart_speed, lv_palette_darken(LV_PALETTE_GREY, 3), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_chart_speed, lv_palette_lighten(LV_PALETTE_BLUE, 2), LV_PART_TICKS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_chart_speed, 255, LV_PART_TICKS | LV_STATE_DEFAULT);
    ui_chart_speed_series = lv_chart_add_series(ui_chart_speed, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y); 

    /* 位置控制界面 */
    panel = lv_obj_create(ui_pages[UI_PAGE_MOTOR_POS]);
    lv_obj_set_size(panel, LV_PCT(100), 50);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    label = lv_label_create(panel);
    lv_label_set_text(label, "目标位置");
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_set_pos = lv_spinbox_create(panel);
    lv_obj_set_width(ui_set_pos, 80);
    lv_obj_set_height(ui_set_pos, 40);
    lv_obj_set_align(ui_set_pos, LV_ALIGN_CENTER);
    lv_spinbox_set_digit_format(ui_set_pos, 3, 0);
    lv_spinbox_set_range(ui_set_pos, 0, 360);
    lv_spinbox_set_value(ui_set_pos, 0);

    /* 新ID设置界面 */
    panel = lv_obj_create(ui_pages[UI_PAGE_SET_ID]);
    lv_obj_set_size(panel, LV_PCT(100), 50);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(panel, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    label = lv_label_create(panel);
    lv_label_set_text(label, "ID");
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);
    ui_set_id = lv_spinbox_create(panel);
    lv_obj_set_width(ui_set_id, 40);
    lv_obj_set_height(ui_set_id, 40);
    lv_obj_set_align(ui_set_id, LV_ALIGN_CENTER);
    lv_spinbox_set_digit_format(ui_set_id, 1, 0);
    lv_spinbox_set_range(ui_set_id, 1, 8);
    lv_spinbox_set_value(ui_set_id, 1);
    lv_obj_set_style_bg_opa(ui_set_id, LV_OPA_TRANSP, LV_PART_CURSOR | LV_STATE_DEFAULT);   // 隐藏光标
    lv_obj_set_style_text_opa(ui_set_id, LV_OPA_TRANSP, LV_PART_CURSOR | LV_STATE_DEFAULT); // 隐藏光标
    ui_set_id_tip = lv_label_create(panel);
    lv_label_set_text(ui_set_id_tip, "按键1修改ID");
    lv_obj_set_align(ui_set_id_tip, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(ui_set_id_tip, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);
    
    now_page=UI_PAGE_CONNECT_DEV;
    select_page=ui_pages[UI_PAGE_CONNECT_DEV];
    last_page=ui_pages[UI_PAGE_CONNECT_DEV];
    lv_obj_set_style_border_color(select_page, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    if (_motor == NULL)
    {
        _motor = motor::Register(lv_spinbox_get_value(ui_dev_id));
    }
    _timer = lv_timer_create(timer_handler, 300, NULL);
}

void gui_motor_focus()
{
    lv_group_focus_obj(parent_root);
}

void gui_motor_del(void)
{
    lv_timer_del(_timer);
}