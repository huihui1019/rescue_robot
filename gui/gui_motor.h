#ifndef _GUI_MOTOR_H_
#define _GUI_MOTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_motor_init(lv_obj_t* root);
void gui_motor_focus(void);
void gui_motor_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_MOTOR_H_ */