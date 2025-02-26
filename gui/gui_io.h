#ifndef _GUI_IO_H_
#define _GUI_IO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_io_init(lv_obj_t* root);
void gui_io_focus(void);
void gui_io_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_IO_H_ */