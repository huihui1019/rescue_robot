#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

extern void setup(void);
extern void loop(void);

/**
 * @brief Arduino风格main函数
 * 
 * @return int 
 */
int main(void)
{
    setup();

    while (true)
    {
        loop();
    }

    return RT_EOK;
}


