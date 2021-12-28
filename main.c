

#include <rtthread.h>
#include "board.h"
#include"myiic.h"
#include"bmp280.h"
#include"myiic2.h"
#include"bmp280_2.h"
#include "uart2.h"
#include"gpio_dev.h"

int main(void)
{
    uint8_t pin = rt_pin_get("PE.1");

    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    rt_pin_write(pin, PIN_HIGH);
    gpio_dev_init();            //电磁阀、风机驱动
    rt_thread_mdelay(2000);
    uart2_init();               //串口初始化
    command_proc_init();        //串口屏命令解析
    IIC_Init();                 //IIC总线驱动
    IIC2_Init();                //IIC2总线驱动
    Bmp1_Init(BMP280_ADDRESS);  //总线1上两个BMP280驱动
    if (line1sensor_num>=1) bmpline1_proc();
    Bmp2_Init(BMP280_ADDRESS);  //总线2上两个BMP280驱动
    if (line2sensor_num>=1) bmpline2_proc();
    rt_thread_mdelay(50);
    while (1)
    {

        rt_thread_mdelay(500);
    }
}
