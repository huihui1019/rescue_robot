/**
 * @file lv_port_disp.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>
#include "lcd.h"
#define LCD_W 240
#define LCD_H 240

/*********************
 *      DEFINES
 *********************/
#define MY_DISP_HOR_RES         (LCD_W)
#define MY_DISP_VER_RES         (LCD_H)
#define SCREEN_BUFFER_SIZE      (MY_DISP_HOR_RES * 10)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    // static lv_disp_draw_buf_t disp_buf;
    // static lv_color_t buf_1[SCREEN_BUFFER_SIZE];                          /*A buffer for 10 rows*/
    // lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, SCREEN_BUFFER_SIZE);   /*Initialize the display buffer*/

    /* Example for 2) */
    static lv_disp_draw_buf_t disp_buf;
    static lv_color_t buf_2_1[SCREEN_BUFFER_SIZE];                        /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[SCREEN_BUFFER_SIZE];                        /*An other buffer for 10 rows*/
    lv_disp_draw_buf_init(&disp_buf, buf_2_1, buf_2_2, SCREEN_BUFFER_SIZE);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &disp_buf;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    lcd_display_on();
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        lcd_fill_color(area->x1, area->y1, area->x2, area->y2, &color_p->full);
    
    }

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}



#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
void lv_port_disp_init(void)
{
}
#endif
