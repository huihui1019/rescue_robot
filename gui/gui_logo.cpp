#include "gui_logo.h"
#include "robot_config.h"

static lv_obj_t *parent_root;

LV_IMG_DECLARE(logo_icon);

static void _ready_cb(lv_anim_t *anim) {
  lv_anim_del(anim, NULL);
  page_fast_load(PAGE_MAIN, true);
}

static void logo_anim_start() {
  lv_obj_t *logo;
  lv_obj_t *block;
  lv_obj_t *label;

  logo = lv_img_create(parent_root);
  lv_img_set_src(logo, &logo_icon);
  lv_obj_align(logo, LV_ALIGN_CENTER, 0, 0);

  label = lv_label_create(parent_root);
  lv_obj_set_x(label, 70);
  lv_obj_set_y(label, 40);
  lv_obj_set_align(label, LV_ALIGN_CENTER);
  lv_label_set_text(label, ROBOT_VERSIONS);

  block = lv_obj_create(parent_root);
  lv_obj_set_size(block, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(block, lv_color_hex(0xf5f5f5),
                            LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_color(block, lv_color_hex(0xf5f5f5),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_align(block, LV_ALIGN_CENTER, 0, 0);

  lv_anim_t anim;
  lv_anim_init(&anim);
  lv_anim_set_var(&anim, block);
  lv_anim_set_values(&anim, 0, 300);
  lv_anim_set_time(&anim, 2000);
  lv_anim_set_ready_cb(&anim, _ready_cb);
  lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_x);
  lv_anim_start(&anim);
}

void gui_logo_init(lv_obj_t *root) {
  parent_root = root;
  lv_obj_clear_flag(parent_root, LV_OBJ_FLAG_SCROLLABLE);
}

void gui_logo_focus() { logo_anim_start(); }

void gui_logo_del(void) {}
