#include "gui_io.h"
#include "robot.h"
#include "keypad.h"

LV_FONT_DECLARE(ui_font_dingtalk);

static lv_obj_t *parent_root;
static lv_timer_t *_timer;
static lv_obj_t *ui_sensor_led[4];
static lv_obj_t *ui_per_led_select;
static lv_obj_t *ui_per_led[6];
static lv_coord_t ui_per_led_pos[6][2] = {{-90, 10},{-55, 10},{-20, 10}, 
                                    {20, 10},{55, 10},{90, 10}};
static int per_led_cnt = 0;

static void _anim(int32_t start, int32_t end)
{
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, ui_per_led_select);
    lv_anim_set_time(&anim, 300);
    lv_anim_set_values(&anim, start, end);
    lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_start(&anim);
}

static void event_key_handler(lv_event_t *e)
{
    if (e->code == LV_EVENT_KEY)
    {
        const uint32_t key = lv_indev_get_key(lv_indev_get_act());

        if (key == KEY1_PRESS_DOWN)
        {
            page_fast_load(PAGE_MAIN, false);
        }
        else if (key == KEY2_PRESS_DOWN)
        {
            page_fast_load(PAGE_IMU, true);
        }
        
        if (key == KEY5_PRESS_DOWN && per_led_cnt < 5)
        {
            per_led_cnt ++;
            _anim(ui_per_led_pos[per_led_cnt-1][0], ui_per_led_pos[per_led_cnt][0]);
        }
        else if (key == KEY4_PRESS_DOWN && per_led_cnt > 0)
        {
            per_led_cnt --;
            _anim(ui_per_led_pos[per_led_cnt+1][0], ui_per_led_pos[per_led_cnt][0]);
        }
        else if (key == KEY3_PRESS_DOWN)
        {
            switch (per_led_cnt)
            {
            case 0:
                robot.SGs.SG1_PWR = !robot.SGs.SG1_PWR;
                robot.SGs.SG1_IO = !robot.SGs.SG1_IO;
                break;
            case 1:
                robot.SGs.SG2_PWR = !robot.SGs.SG2_PWR;
                robot.SGs.SG2_IO = !robot.SGs.SG2_IO;
                break;
            case 2:
                robot.SGs.SG3_PWR = !robot.SGs.SG3_PWR;
                robot.SGs.SG3_IO = !robot.SGs.SG3_IO;
                break;
            case 3:
                robot.SGs.SG4_PWR = !robot.SGs.SG4_PWR;
                robot.SGs.SG4_IO = !robot.SGs.SG4_IO;
                break;
            case 4:
                robot.SGs.SG5_PWR = !robot.SGs.SG5_PWR;
                robot.SGs.SG5_IO = !robot.SGs.SG5_IO;
                break;
            case 5:
                robot.SGs.SG6_PWR = !robot.SGs.SG6_PWR;
                robot.SGs.SG6_IO = !robot.SGs.SG6_IO;
                break;
            
            default:
                break;
            }
            lv_led_toggle(ui_per_led[per_led_cnt]);
        }
    }
}

static void timer_handler(lv_timer_t * timer)
{
    lv_led_set_brightness(ui_sensor_led[0], io::getIN1() ? LV_LED_BRIGHT_MAX:LV_LED_BRIGHT_MIN);
    lv_led_set_brightness(ui_sensor_led[1], io::getIN2() ? LV_LED_BRIGHT_MAX:LV_LED_BRIGHT_MIN);
    lv_led_set_brightness(ui_sensor_led[2], io::getIN3() ? LV_LED_BRIGHT_MAX:LV_LED_BRIGHT_MIN);
    lv_led_set_brightness(ui_sensor_led[3], io::getIN4() ? LV_LED_BRIGHT_MAX:LV_LED_BRIGHT_MIN);

}

void gui_io_init(lv_obj_t* root)
{
    lv_obj_t *label;
    lv_obj_t *panel;

    per_led_cnt = 0;
    rt_memset(&robot.SGs, 0, sizeof(switchgear_t));
    parent_root = root;
    lv_group_add_obj(lv_group_get_default(), parent_root);
    lv_obj_add_event_cb(parent_root, event_key_handler, LV_EVENT_KEY, NULL);

    label = lv_label_create(parent_root);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 10);
    lv_obj_set_align(label, LV_ALIGN_TOP_MID);
    lv_label_set_text(label, "IO调试界面");
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
    lv_obj_set_width(panel, 230);
    lv_obj_set_height(panel, 120);
    lv_obj_set_x(panel, 0);
    lv_obj_set_y(panel, -25);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    label = lv_label_create(panel);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 0);
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_label_set_text(label, "传感器");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFBFBFB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_sensor_led[0] = lv_led_create(panel);
    lv_obj_set_width(ui_sensor_led[0], 40);
    lv_obj_set_height(ui_sensor_led[0], 20);
    lv_obj_set_x(ui_sensor_led[0], -50);
    lv_obj_set_y(ui_sensor_led[0], -40);
    lv_led_set_color(ui_sensor_led[0], lv_palette_main(LV_PALETTE_GREEN));
    lv_obj_set_align(ui_sensor_led[0], LV_ALIGN_CENTER);
    lv_led_off(ui_sensor_led[0]);

    ui_sensor_led[1] = lv_led_create(panel);
    lv_obj_set_width(ui_sensor_led[1], 20);
    lv_obj_set_height(ui_sensor_led[1], 40);
    lv_obj_set_x(ui_sensor_led[1], -90);
    lv_obj_set_y(ui_sensor_led[1], -20);
    lv_led_set_color(ui_sensor_led[1], lv_palette_main(LV_PALETTE_GREEN));
    lv_obj_set_align(ui_sensor_led[1], LV_ALIGN_CENTER);
    lv_led_off(ui_sensor_led[1]);

    ui_sensor_led[2] = lv_led_create(panel);
    lv_obj_set_width(ui_sensor_led[2], 20);
    lv_obj_set_height(ui_sensor_led[2], 40);
    lv_obj_set_x(ui_sensor_led[2], -90);
    lv_obj_set_y(ui_sensor_led[2], 30);
    lv_led_set_color(ui_sensor_led[2], lv_palette_main(LV_PALETTE_GREEN));
    lv_obj_set_align(ui_sensor_led[2], LV_ALIGN_CENTER);
    lv_led_off(ui_sensor_led[2]);

    ui_sensor_led[3] = lv_led_create(panel);
    lv_obj_set_width(ui_sensor_led[3], 40);
    lv_obj_set_height(ui_sensor_led[3], 20);
    lv_obj_set_x(ui_sensor_led[3], 75);
    lv_obj_set_y(ui_sensor_led[3], -40);
    lv_led_set_color(ui_sensor_led[3], lv_palette_main(LV_PALETTE_GREEN));
    lv_obj_set_align(ui_sensor_led[3], LV_ALIGN_CENTER);
    lv_led_off(ui_sensor_led[3]);
    
    panel = lv_obj_create(parent_root);
    lv_obj_set_width(panel, 230);
    lv_obj_set_height(panel, 60);
    lv_obj_set_x(panel, 0);
    lv_obj_set_y(panel, 70);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    label = lv_label_create(panel);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, -15);
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_label_set_text(label, "外设");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFBFBFB), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_per_led_select = lv_obj_create(panel);
    lv_obj_set_x(ui_per_led_select, -90);
    lv_obj_set_y(ui_per_led_select, 10);
    lv_obj_set_width(ui_per_led_select, 40);
    lv_obj_set_height(ui_per_led_select, 30);
    lv_obj_set_align(ui_per_led_select, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_per_led_select, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_per_led_select, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_per_led_select, lv_color_hex(0xC0FF6B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_per_led_select, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    for (uint8_t i=0; i<6; i++)
    {
        ui_per_led[i] = lv_led_create(panel);
        lv_obj_set_width(ui_per_led[i], 30);
        lv_obj_set_height(ui_per_led[i], 20);
        lv_obj_set_x(ui_per_led[i], ui_per_led_pos[i][0]);
        lv_obj_set_y(ui_per_led[i], ui_per_led_pos[i][1]);
        lv_obj_set_align(ui_per_led[i], LV_ALIGN_CENTER);
        lv_led_off(ui_per_led[i]);
    }

    _timer = lv_timer_create(timer_handler, 300, NULL);
}

void gui_io_focus()
{
    lv_group_focus_obj(parent_root);
}

void gui_io_del(void)
{
    lv_timer_del(_timer);
}