#ifndef _GUI_MAIN_H_
#define _GUI_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_main_init(lv_obj_t* root);
void gui_main_focus(void);
void gui_main_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_MAIN_H_ */