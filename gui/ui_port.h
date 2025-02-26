#ifndef _UI_PORT_H_
#define _UI_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

typedef void(*page_init_handle)(lv_obj_t* root);
typedef void(*page_focus_handle)(void);
typedef void(*page_del_handle)(void);

typedef struct page_list
{
    uint8_t id;
    lv_obj_t* root;
    bool is_init;
    page_init_handle init_handler;
    page_focus_handle focus_handler;
    page_del_handle del_handler;
    struct page_list *next_page;
}page_list_t;

#ifdef __cplusplus
}
#endif

#endif /* _UI_PORT_H_ */
