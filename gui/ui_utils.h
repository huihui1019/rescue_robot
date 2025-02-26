#ifndef _UI_UTILS_H_
#define _UI_UTILS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"

void lv_obj_add_anim(lv_obj_t* obj, lv_anim_exec_xcb_t exec_cb, uint16_t time, lv_coord_t start, lv_coord_t end, lv_anim_path_cb_t path_cb);
lv_obj_t *create_page(lv_obj_t* parent, const char * title);
void pages_switching(lv_obj_t *select_page, lv_obj_t *last_page, bool select);
void spinbox_controls(lv_obj_t *spinbox_obj, bool increment, bool decrement, bool step_next, bool step_prev);

#ifdef __cplusplus
}
#endif

#endif /* _UI_UTILS_H_ */
