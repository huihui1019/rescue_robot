#ifndef _GUI_LOGO_H_
#define _GUI_LOGO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_logo_init(lv_obj_t* root);
void gui_logo_focus(void);
void gui_logo_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_LOGO_H_ */