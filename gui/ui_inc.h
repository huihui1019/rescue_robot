#ifndef _UI_INC_H_
#define _UI_INC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"
#include "ui_utils.h"
// #include "key_char.h"

enum GUI_PAGE
{
    PAGE_MAIN = 0,
    PAGE_IO,
    PAGE_IMU,
    PAGE_MOTOR,
    PAGE_SERVO,
    PAGE_UART,
    PAGE_LOGO,
    PAGE_TOTAL_NUM
};

typedef void(*page_call_handle)(void* arg);

#define UI_DEBUG 0
#if UI_DEBUG
#define UI_PRINTF       rt_kprintf
#else
#define UI_PRINTF(...)
#endif

#define gui_delay(t){\
    int _t = t;\
    while(_t--) \
    { \
        lv_task_handler();\
        rt_thread_mdelay(1);\
    }\
}

#define PAGE_INIT_DEF(PAGE)  gui_##PAGE##_init
#define PAGE_FOCUS_DEF(PAGE) gui_##PAGE##_focus
#define PAGE_DEL_DEF(PAGE) gui_##PAGE##_del

#define ANIMATION_TIME 250	//切换页面的动画时间

void page_load(uint8_t id, lv_scr_load_anim_t anim_type, uint32_t time, uint32_t delay, bool auto_del);
void page_fast_load(uint8_t id, bool auto_del);

#ifdef __cplusplus
}
#endif

#endif /* _UI_INC_H_ */
