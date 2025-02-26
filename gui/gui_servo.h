#ifndef _GUI_SERVO_H_
#define _GUI_SERVO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_servo_init(lv_obj_t* root);
void gui_servo_focus(void);
void gui_servo_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_SERVO_H_ */