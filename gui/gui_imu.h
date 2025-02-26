#ifndef _GUI_IMU_H_
#define _GUI_IMU_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ui_inc.h"

void gui_imu_init(lv_obj_t* root);
void gui_imu_focus(void);
void gui_imu_del(void);

#ifdef __cplusplus
}
#endif

#endif /* _GUI_IMU_H_ */