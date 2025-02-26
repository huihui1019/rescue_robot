from building import *
import os
import rtconfig

cwd  = GetCurrentDir()

group = []
path = [cwd + '/inc']
src  = Glob('src/*.c') + Glob('src/*.cpp')

if GetDepend(['PKG_USING_LVGL']):
    lvgl_port_src = Glob('gui/lvgl_port/*.c')
    lvgl_port_inc = [cwd + '/gui/lvgl_port']
    group += DefineGroup('LVGL-port', lvgl_port_src, depend = ['PKG_USING_LVGL'], CPPPATH = lvgl_port_inc)

    gui_src = Glob('gui/*.c') + Glob('gui/*.cpp')
    gui_inc = [cwd + '/gui']
    group += DefineGroup('Applications/GUI', gui_src, depend = ['PKG_USING_LVGL'], CPPPATH = gui_inc)

    resource_src = Glob('gui/resource/*.c')
    resource_inc = [cwd + 'gui/resource']
    group += DefineGroup('Applications/Resource', resource_src, depend = ['PKG_USING_LVGL'], CPPPATH = resource_inc)


CXXFLAGS = ''
LOCAL_CCFLAGS = ''
LOCAL_CFLAGS = ''
LOCAL_CXXFLAGS = ''


if rtconfig.PLATFORM in ['gcc', 'armclang', 'llvm-arm']: # GCC or Keil AC6 or Clang/LLVM
    CXXFLAGS += ' -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections -Wl,--gc-sections' # reduce resource consumption
    LOCAL_CFLAGS += ' -std=c99' # enable GNU extension. Cannot use -std=c99, some toolchain like RISC-V GCC doesn't support 'asm' key word
    LOCAL_CXXFLAGS += ' -std=c++11' # support C++11, like non-static data member initializers

elif rtconfig.PLATFORM in ['armcc']: # Keil AC5
    CXXFLAGS += ' --gnu --c99' # enable global C99 and GNU extension support for the whole project
    LOCAL_CCFLAGS += ' --gnu -g -W'
    LOCAL_CFLAGS += ' --c99' # cannot use --c99 symbol for C++ files, pertically in Keil
    LOCAL_CXXFLAGS += ' --cpp11' # support C++11
else:
    print('[applications] Unsupported rtconfig.PLATFORM: {}'.format(rtconfig.PLATFORM))

group += DefineGroup('Applications', src,  depend = ['APP_RESCUE_ROBOT'], CPPPATH = path, 
                    CXXFLAGS=CXXFLAGS,
                    LOCAL_CCFLAGS=LOCAL_CCFLAGS,
                    LOCAL_CFLAGS=LOCAL_CFLAGS,
                    LOCAL_CXXFLAGS=LOCAL_CXXFLAGS)


Return('group')
