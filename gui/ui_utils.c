#include "ui_utils.h"

LV_FONT_DECLARE(ui_font_dingtalk);

void lv_obj_add_anim(lv_obj_t* obj, lv_anim_exec_xcb_t exec_cb, uint16_t time, lv_coord_t start, lv_coord_t end, lv_anim_path_cb_t path_cb)
{
    lv_anim_t a;

    lv_anim_init(&a);
    lv_anim_set_var(&a, obj); //动画对象

    lv_anim_set_exec_cb(&a, exec_cb); //动画函数
    lv_anim_set_time(&a, time);

    lv_anim_set_values(&a, start, end); //起始值 结束值
    lv_anim_set_path_cb(&a, path_cb); //动画计算方法
    lv_anim_set_playback_time(&a, 0); //回放时间设为0不执行动画回放			 

    lv_anim_start(&a); //开启动画
}

// void lv_obj_add_anim()
// {
//     lv_anim_t a;                                       //创建动画样式
//    lv_anim_init(&a);                                  //初始化动画
//    lv_anim_set_var(&a,obj);                           //给动画设置一个变量
//    lv_anim_set_values(&a,10,50);                      //设置一个动画值
//    lv_anim_set_time(&a,1000);                         //设置动画时间
//    lv_anim_set_playback_delay(&a,100);                //回放延时 使动画回放时，正向方向准备好了
//    lv_anim_set_playback_time(&a,300);                 //回放时间
//    lv_anim_set_repeat_delay(&a,500);                  //重复延时
//    lv_anim_set_repeat_count(&a,LV_ANIM_REPEAT_INFINITE); //重复计数次数
//    lv_anim_set_path_cb(&a,lv_anim_path_ease_in_out);  //设置动画播放路径

//    lv_anim_set_exec_cb(&a,anim_size_cb);              //给动画设置一个功能 回调函数为尺寸
//    lv_anim_start(&a);                                 //开始动画
//    lv_anim_set_exec_cb(&a,anim_x_cb);                 //给动画设置一个功能 回调函数为x轴值
//    lv_anim_set_values(&a,10,240);                     //给动画设置一个值
//    lv_anim_start(&a);                                 //开始动画
// }

void lv_label_set_num(lv_obj_t* obj, int16_t num)
{
    lv_label_set_text_fmt(obj, "%d", num);
}

void lv_label_set_float(lv_obj_t* obj, float num)
{
    lv_label_set_text_fmt(obj, "%f", num);
}

lv_obj_t *create_page(lv_obj_t* parent, const char * title)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_align(panel, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_border_color(panel, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 5, LV_PART_MAIN);
    lv_obj_set_style_bg_color(panel, lv_palette_lighten(LV_PALETTE_RED, 2), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(panel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(panel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(panel, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(panel, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *label = lv_label_create(panel);
    lv_label_set_text(label, title);
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 0);
    lv_obj_set_align(label, LV_ALIGN_TOP_LEFT);
    lv_obj_set_style_text_font(label, &ui_font_dingtalk, LV_PART_MAIN | LV_STATE_DEFAULT);
    return panel;
}

void pages_switching(lv_obj_t *select_page, lv_obj_t *last_page, bool select)
{
    if (select_page != last_page)
    {
        lv_obj_set_style_border_color(last_page, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_scroll_to_view(select_page, LV_ANIM_ON);
    }

    if (select)
    {
        lv_obj_set_style_border_color(select_page, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    }
    else
    {
        lv_obj_set_style_border_color(select_page, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN);
    }
}

void spinbox_controls(lv_obj_t *spinbox_obj, bool increment, bool decrement, bool step_next, bool step_prev)
{
    if (increment)
    {
        lv_spinbox_increment(spinbox_obj);
    }

    if (decrement)
    {
        lv_spinbox_decrement(spinbox_obj);
    }

    if (step_next)
    {
        lv_spinbox_step_next(spinbox_obj);
    }

    if (step_prev)
    {
        lv_spinbox_step_prev(spinbox_obj);
    }
}
