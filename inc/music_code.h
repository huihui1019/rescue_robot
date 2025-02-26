#ifndef __MUSIC_CODE_H__
#define __MUSIC_CODE_H__

#include "tone_player.hpp"

/*开机音*/
MUSIC_DEF(Startup){
    {L5, 200, 3}, {0, 50, 3}, {L4, 200, 3}, {0, 50, 3},
    {L3, 200, 3}, {0, 50, 3}, {L5, 200, 3},
};

/*关机音*/
MUSIC_DEF(Shutdown){
    {H6, 80, 3},
    {M6, 80, 3},
    {L6, 80, 3},
};

/*错误提示音*/
MUSIC_DEF(Error){
    {100, 80, 3},
    {0, 80, 3},
    {100, 80, 3},
};

/*连接成功音*/
MUSIC_DEF(Connect){
    {H1, 80, 3},
    {H2, 80, 3},
    {H3, 80, 3},
};

/*断开连接音*/
MUSIC_DEF(Disconnect){
    {H3, 80, 3},
    {H2, 80, 3},
    {H1, 80, 3},
};

/*充电提示音*/
MUSIC_DEF(BattCharge){
    {M3, 100, 3},
    {0, 100, 3},
    {L3, 100, 3},
    {0, 100, 3},
};

MUSIC_DEF(ManualCtrl){
    {M1, 100, 3},
    {0, 50, 3},

};

MUSIC_DEF(AutoCtrl){
    {M1, 100, 3},
    {0, 50, 3},
    {M1, 100, 3},
    {0, 50, 3},
};

MUSIC_DEF(SetEnv){
    {M7, 100, 3},
};

/*提示音列表*/
static const MusicList_t MusicList[] = {
    ADD_MUSIC(Startup),    ADD_MUSIC(Shutdown),   ADD_MUSIC(Error),
    ADD_MUSIC(Connect),    ADD_MUSIC(Disconnect), ADD_MUSIC(BattCharge),
    ADD_MUSIC(ManualCtrl), ADD_MUSIC(AutoCtrl),   ADD_MUSIC(SetEnv),
};

#endif
