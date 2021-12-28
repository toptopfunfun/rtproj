/*
 * Copyright (c) 2006-2021, BeiJingJiaDun Development Team
   *  串口2接收数据并处理
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-10     topfun       the first version
 */

#include <uart2.h>
#include "stdio.h"
#include "gpio_dev.h"
#define SAMPLE_UART_NAME       "uart1"//本来用2，此处改为1

/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
static rt_device_t serial;
char scr_comm[7];
u_int8_t hl1,hl2;
/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    return RT_EOK;
}

void exc_commd(void *parameter)//根据scr_comm命令缓存执行命令
{
//    unsigned char scrcomm_backadd[]={0xff,0xff,0xff};
    unsigned char comm_con[] = {0x70,0x61,0x67,0x65,0x20,0x70,0x61,0x67,0x65,0x5f,0x72,0x75,0x6e,0xff,0xff,0xff};//指令确认
    //page page_run               70   61   67   65   20   70   61   67   65   5F   72   75   6E
    //                            70   61   67   65   20   70   61   67   65   5F   72   75   6E
    hl1=1;
    scr_comm[0]=0;
    scr_comm[1]=0;
    scr_comm[2]=0;
    scr_comm[3]=0;
    scr_comm[4]=0;
    scr_comm[5]=0;
    scr_comm[6]=0;
while(1)
{
    while(scr_comm[6]==0x0d)    //如果scr_comm[6]==0x0d表示接收到一个完整的命令
    {
        if ((scr_comm[3]==0x62)&&(scr_comm[4]==0x30)&&(scr_comm[5]==0x30))//b00:启动反吹
        {
            scr_comm[6]=0;
            rt_kprintf("Open blowback!\n");
            uart2_send_val(comm_con,16);
            //uart2_send_val(scrcomm_backadd,3);
            rt_thread_mdelay(10);
            if (hl1==1)
            {
                rt_kprintf("Open valve1!\n");
                rt_pin_write(Bb_Valve1, PIN_HIGH);//打开1回路阀门
                rt_thread_mdelay(12000);
                //等待完全打开
                rt_kprintf("Open Fan!\n");
                rt_pin_write(Fan,PIN_HIGH);       //打开风机
                rt_thread_mdelay(5000);           //反吹时间
                rt_kprintf("Close Fan!\n");
                rt_pin_write(Fan,PIN_LOW);        //关闭风机
                rt_kprintf("Close valve1!\n");
                rt_pin_write(Bb_Valve1, PIN_LOW); //关闭阀门
            }
//            if (hl2==1)
//                rt_pin_write(Bb_Valve2, PIN_HIGH);


//            rt_pin_write(Fan,PIN_LOW);
//
//            rt_pin_write(Bb_Valve2, PIN_LOW);

            scr_comm[0]=0;
            scr_comm[1]=0;
            scr_comm[2]=0;
            scr_comm[3]=0;
            scr_comm[4]=0;
            scr_comm[5]=0;
            scr_comm[6]=0;
//            rt_kprintf("Auto close blowback!\n");
         }
//        if ((scr_comm[3]==0x74)&&(scr_comm[4]==0x35)&&(scr_comm[5]==0x30))//t50:回路1关闭
//          hl1=0;
//        if ((scr_comm[3]==0x74)&&(scr_comm[4]==0x35)&&(scr_comm[5]==0x31))//t51:回路1打开
//          hl1=1;
//        if ((scr_comm[3]==0x74)&&(scr_comm[4]==0x37)&&(scr_comm[5]==0x30))//t70:回路1关闭
//          hl2=0;
//        if ((scr_comm[3]==0x74)&&(scr_comm[4]==0x37)&&(scr_comm[5]==0x31))//t71:回路1打开
//          hl2=1;
        scr_comm[0]=0;
        scr_comm[1]=0;
        scr_comm[2]=0;
        scr_comm[3]=0;
        scr_comm[4]=0;
        scr_comm[5]=0;
        scr_comm[6]=0;

    }
    rt_thread_mdelay(50);//如果没有延迟，在控制台会无法输入命令
}
}
void command_proc_init(void)
{
    rt_thread_t thread = rt_thread_create(          /* 创建 exc_cmmd 线程 */
                                        "exc_cmmd",
                                        exc_commd,
                                        RT_NULL,
                                        1024,
                                        5,
                                        30);
    if (thread != RT_NULL) {                     /* 创建成功则启动线程 */
        rt_thread_startup(thread);
    }
    else {
        rt_kprintf("Create %s Entry failed!\n");
    }
}

static void serial_thread_entry(void *parameter)
{
    char ch;                //接收的字节
    u_int8_t p_scrcom=0;    //接收的屏幕命令缓冲区指针
    while (1)
    {
        while (rt_device_read(serial, -1, &ch, 1) != 1)  /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        {
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);    /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
        }

        switch(p_scrcom)    //对应命令缓冲区的位置，判断是否是正确的字节，不是则丢弃重新接收
        {
        case 0:
            if(ch==0x73)    //第一个字节应该是s，是则放入缓冲区
            {
                scr_comm[p_scrcom]=ch;  //
                p_scrcom++;             //同时准备接收下一个字节
                break;
            }
            else {
                p_scrcom=0;
                scr_comm[6]=0;
                break;
            }
        case 1:
            if(ch==0x63)//c
            {
                scr_comm[p_scrcom]=ch;
                p_scrcom++;
                break;
            }
            else {
                p_scrcom=0;
                scr_comm[6]=0;
                break;
            }
        case 2:
            if(ch==0x72)//r
            {
                scr_comm[p_scrcom]=ch;
                p_scrcom++;
                break;
            }
            else {
                p_scrcom=0;
                scr_comm[6]=0;
                break;
            }
        case 3:         //scr之后，第4个字符，全收，不做判断

                scr_comm[p_scrcom]=ch;
                p_scrcom++;
                break;


        case 4://收取第5个字符，不判断
                scr_comm[p_scrcom]=ch;
                p_scrcom++;
                break;
        case 5://收取第6个字符，判断是不是b40,如果是b40,则是关闭风机。
               if ((scr_comm[3]==0x62)&&(scr_comm[4]==0x34)&&(ch==0x30))
               {
                   rt_pin_write(Fan,PIN_LOW);
                   rt_pin_write(Bb_Valve1, PIN_LOW);
//                   rt_pin_write(Bb_Valve2, PIN_LOW);
                   p_scrcom=0;
                   scr_comm[6]=0;
                   break;
               }
               scr_comm[p_scrcom]=ch;
               p_scrcom++;
               break;

        case 6:
            if(ch==0x0d)
            {
                scr_comm[p_scrcom]=ch;
                p_scrcom=0;
                break;
            }

        }
        rt_thread_mdelay(50);
    }
}

void uart2_init(void)
{
       serial = rt_device_find(SAMPLE_UART_NAME);           /* 查找系统中的串口设备 */
       if (!serial)
       {
           rt_kprintf("find %s failed!\n", SAMPLE_UART_NAME);
       }
       rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);        /* 初始化信号量 */
       rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);              /* 以中断接收及轮询发送模式打开串口设备 */
       rt_device_set_rx_indicate(serial, uart_input);              /* 设置接收回调函数 */
       //rt_device_write(serial, 0, str, (sizeof(str) - 1));       /* 发送字符串 */

       rt_thread_t thread = rt_thread_create(                      /* 创建 serial 线程 */
                                           "serial",
                                           serial_thread_entry,
                                           RT_NULL,
                                           1024,
                                           4,
                                           30);
       if (thread != RT_NULL)               /* 创建成功则启动线程 */
       {
           rt_thread_startup(thread);
       }
       else
       {
           rt_kprintf("Create %s Entry failed!\n", SAMPLE_UART_NAME);
       }
}
void uart2_send_val(unsigned char str[],u_int8_t lengh)
{
    rt_enter_critical();

    rt_device_write(serial, 0, str, lengh);
    /* 退出临界段 */
    rt_exit_critical();

}


