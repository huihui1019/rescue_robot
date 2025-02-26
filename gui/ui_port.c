#include "ui_port.h"
#include "gui_main.h"
#include "gui_io.h"
#include "gui_imu.h"
#include "gui_motor.h"
#include "gui_servo.h"
#include "gui_logo.h"
#include "gui_uart.h"

static page_list_t *phead = NULL;      //链表头
static page_list_t *last_page = NULL;
lv_style_t style_transp;
page_call_handle page_call_handler;
lv_group_t *group;

extern lv_indev_t * indev_keypad;

static page_list_t* page_foreach(void)
{
    page_list_t *tmp = phead;
    while(tmp->next_page)
    {
        tmp = tmp->next_page;
        UI_PRINTF("page_id: %d\n", tmp->id);
    }

    return tmp;
}

static void page_add(uint8_t id, page_init_handle init_handler, page_focus_handle focus_handler, page_del_handle del_handler)
{
    page_list_t *tmp = phead;
    page_list_t *insert = NULL;
    
    //尾部插入
    while(tmp->next_page) tmp = tmp->next_page;

    insert = (page_list_t*)LV_MEM_CUSTOM_ALLOC(sizeof(page_list_t));
    if(insert == NULL)
        LV_ASSERT_HANDLER;
    memset(insert, 0, sizeof(page_list_t));
    
    tmp->next_page = insert;
    insert->next_page = NULL;
    
    insert->id = id;
    insert->init_handler = init_handler;
    insert->focus_handler = focus_handler;
    insert->del_handler = del_handler;
}

static void page_remove(uint8_t id)
{
    page_list_t *tmp = phead;
    page_list_t *prev = tmp;
    page_list_t *del = NULL;
    
    while(tmp)
    {
        tmp = tmp->next_page;
        if(tmp->id == id)
        {			
            del = tmp;
            prev->next_page = tmp->next_page;
            break;
        }
        
        prev = tmp;
    }
}

static void page_clear()
{
    page_list_t *tmp = phead, *del = NULL;
    
    while(tmp)
    {
        del = tmp->next_page;
        tmp->next_page = del->next_page;
        LV_MEM_CUSTOM_FREE(del);
    }
}

static page_list_t* page_find(uint8_t id)
{
    page_list_t *tmp = phead;
    
    while(tmp->next_page)
    {
        tmp = tmp->next_page;
        if(tmp->id == id)
        {
            return tmp;
        }
    }
    return NULL;
}

static lv_obj_t* page_init(page_list_t *page)
{
    if(page)
    {
        page->root = lv_obj_create(NULL);
        lv_obj_center(page->root);
        if (page->init_handler != NULL)
            page->init_handler(page->root);
        page->is_init = true;
    }

    return page->root;
}

void page_load(uint8_t id, lv_scr_load_anim_t anim_type, uint32_t time, uint32_t delay, bool auto_del)
{
    page_list_t *page = page_find(id);
    if (page == NULL)
    {
        UI_PRINTF("ID not found\n");
        return;
    }
    else
    {
        UI_PRINTF("page handler: %d\n", page->id);
    }
    lv_obj_t *root = page->root;

    if (!page->is_init)
    {
        root = page_init(page);
    }
    lv_scr_load_anim(root, anim_type, time, delay, auto_del);
    if (auto_del && last_page)
    {
        if (page->del_handler != NULL)
            last_page->del_handler();
        last_page->is_init = false;
    }
    if (page->focus_handler != NULL)
        page->focus_handler();
    last_page = page;
}

void page_fast_load(uint8_t id, bool auto_del)
{
    page_load(id, LV_SCR_LOAD_ANIM_NONE, 0, 0, auto_del);
}

void ui_port_init(void)
{
    phead = (page_list_t*)LV_MEM_CUSTOM_ALLOC(sizeof(page_list_t));
    if(phead == NULL)
    {
        LV_ASSERT_HANDLER;
    }
    memset(phead, 0, sizeof(page_list_t));

    page_add(PAGE_MAIN, PAGE_INIT_DEF(main), PAGE_FOCUS_DEF(main), PAGE_DEL_DEF(main));
    page_add(PAGE_IO, PAGE_INIT_DEF(io), PAGE_FOCUS_DEF(io), PAGE_DEL_DEF(io));
    page_add(PAGE_IMU, PAGE_INIT_DEF(imu), PAGE_FOCUS_DEF(imu), PAGE_DEL_DEF(imu));
    page_add(PAGE_MOTOR, PAGE_INIT_DEF(motor), PAGE_FOCUS_DEF(motor), PAGE_DEL_DEF(motor));
    page_add(PAGE_SERVO, PAGE_INIT_DEF(servo), PAGE_FOCUS_DEF(servo), PAGE_DEL_DEF(servo));
    page_add(PAGE_UART, PAGE_INIT_DEF(uart), PAGE_FOCUS_DEF(uart), PAGE_DEL_DEF(uart));
    page_add(PAGE_LOGO, PAGE_INIT_DEF(logo), PAGE_FOCUS_DEF(logo), PAGE_DEL_DEF(logo));


#if UI_DEBUG
    page_foreach();
#endif

    //初始化透明背景样式
    lv_style_init(&style_transp);
    lv_style_set_bg_opa(&style_transp, LV_OPA_TRANSP);
    lv_style_set_border_opa(&style_transp, LV_OPA_TRANSP);

    //初始化首页
    page_list_t *page = page_find(PAGE_LOGO);
    last_page = page;
    lv_scr_load(page_init(page));
    page->focus_handler();
}

void lv_user_gui_init(void)
{
    //设置按键控制组
    group = lv_group_create();
    lv_indev_set_group(indev_keypad, group); 
    lv_group_set_default(group);

    ui_port_init();
}
