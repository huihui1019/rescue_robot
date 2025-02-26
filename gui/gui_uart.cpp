#include "gui_uart.h"
#include "robot.h"
#include "keypad.h"

LV_FONT_DECLARE(ui_font_dingtalk);

#define TEXTAREA_MAX_LINE 100

static lv_obj_t *parent_root;
static lv_obj_t *ui_group_panel;
static lv_obj_t *ui_textarea;
static lv_timer_t *_timer;

rt_device_t test_uart2 = NULL;
rt_device_t test_uart3 = NULL;
char test_str[20];
uint16_t ui_textarea_show_len = 0;
uint16_t test_len = 0;

static void event_key_handler(lv_event_t *e)
{
    if (e->code == LV_EVENT_KEY)
    {
        const uint32_t key = lv_indev_get_key(lv_indev_get_act());
        
        if (key == KEY1_PRESS_DOWN)
        {
            page_fast_load(PAGE_SERVO, true);
            return;
        }
        // else if (KEY_DOWN(KEY2))
        // {
        //     page_fast_load(PAGE_SERVO, true);
        //     return;
        // }
        else if (key == KEY4_PRESS_DOWN)
        {
            if (test_len == 0)
            {
                test_len = rt_sprintf(test_str, "%s->ok\n",test_uart2->parent.name);
                rt_device_write(test_uart2, 0, test_str, test_len);
            }
        }
        else if (key == KEY5_PRESS_DOWN)
        {
            if (test_len == 0)
            {
                test_len = rt_sprintf(test_str, "%s->ok\n",test_uart3->parent.name);
                rt_device_write(test_uart3, 0, test_str, test_len);
            }
        }
    }
}

static void timer_handler(lv_timer_t * timer)
{
    uint8_t ch;

    while (rt_device_read(test_uart2, -1, &ch, 1) == 1)
    {
        if (test_len > 0)
        {
            test_len --;
        }
        else
        {
            rt_device_write(test_uart2, 0, &ch, 1);
        }
        ui_textarea_show_len++;
        lv_textarea_add_char(ui_textarea, ch);
    }

    while (rt_device_read(test_uart3, -1, &ch, 1) == 1)
    {
        if (test_len > 0)
        {
            test_len --;
        }
        else
        {
            rt_device_write(test_uart3, 0, &ch, 1);
        }
        ui_textarea_show_len++;
        lv_textarea_add_char(ui_textarea, ch);
        
    }
    if (ui_textarea_show_len > TEXTAREA_MAX_LINE)
    {
        ui_textarea_show_len = 0;
        lv_textarea_set_text(ui_textarea, "");
    }

}

void gui_uart_init(lv_obj_t* root)
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
    lv_label_set_text(label, "视觉串口调试界面");
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

    
    // label = lv_label_create(parent_root);
    // lv_obj_set_width(label, LV_SIZE_CONTENT);
    // lv_obj_set_height(label, LV_SIZE_CONTENT);
    // lv_obj_set_x(label, -10);
    // lv_obj_set_y(label, 10);
    // lv_obj_set_align(label, LV_ALIGN_TOP_RIGHT);
    // lv_label_set_text(label, "K2->");
    // lv_obj_set_style_text_color(label, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    label = lv_label_create(parent_root);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, -70);
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_label_set_text(label, "按键4 U2->U3 按键5 U3->U2");
    lv_obj_set_style_text_color(label, lv_color_hex(0x171721), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_textarea = lv_textarea_create(parent_root);
    lv_obj_set_width(ui_textarea, 240);
    lv_obj_set_height(ui_textarea, 175);
    lv_obj_set_x(ui_textarea, 0);
    lv_obj_set_y(ui_textarea, 30);
    lv_obj_set_align(ui_textarea, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_textarea, TEXTAREA_MAX_LINE);

    if (test_uart2 == NULL)
    {
        test_uart2 = rt_device_find("uart2");
    }

    if (test_uart3 == NULL)
    {
        test_uart3 = rt_device_find("uart3");
    }

    if (!(test_uart2->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        rt_device_open(test_uart2, RT_DEVICE_FLAG_INT_RX);
    }

    if (!(test_uart3->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        rt_device_open(test_uart3, RT_DEVICE_FLAG_INT_RX);
    }

    test_len = 0;
    ui_textarea_show_len = 0;
    _timer = lv_timer_create(timer_handler, 100, NULL);

}

void gui_uart_focus(void)
{
    lv_group_focus_obj(parent_root);
    visual::startDebug();
}

void gui_uart_del(void)
{
    visual::stopDebug();
    lv_timer_del(_timer);
}