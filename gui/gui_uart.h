#ifndef _GUI_UART_H_
#define _GUI_UART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_uart_init(lv_obj_t* root);
void gui_uart_focus(void);
void gui_uart_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_UART_H_ */