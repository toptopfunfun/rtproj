/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-28     Administrator       the first version
 */

#include "myiic2.h"
#include "board.h"

//初始化IIC2
void IIC2_Init(void)
{
    rt_pin_mode(IIC2_SCL, PIN_MODE_OUTPUT);
    rt_pin_mode(IIC2_SDA, PIN_MODE_OUTPUT);

    rt_pin_write(IIC2_SCL, PIN_HIGH);
    rt_pin_write(IIC2_SDA, PIN_HIGH);


}
//产生IIC2起始信号
void IIC2_Start(void)
{
    rt_pin_mode(IIC2_SDA, PIN_MODE_OUTPUT);     //sda线输出
    rt_pin_write(IIC2_SDA, PIN_HIGH);
    rt_pin_write(IIC2_SCL, PIN_HIGH);;
    rt_hw_us_delay(8);
    rt_pin_write(IIC2_SDA, PIN_LOW);
    rt_hw_us_delay(8);
    rt_pin_write(IIC2_SCL, PIN_LOW);//钳住I2C总线，准备发送或接收数据
    rt_hw_us_delay(8);
}
//产生IIC2停止信号
void IIC2_Stop(void)
{
    rt_pin_mode(IIC2_SDA, PIN_MODE_OUTPUT);//sda线输出
    rt_pin_write(IIC2_SCL, PIN_LOW);
    rt_pin_write(IIC2_SDA, PIN_LOW);
    rt_hw_us_delay(8);
    rt_pin_write(IIC2_SCL, PIN_HIGH);;
        rt_hw_us_delay(4);
    rt_pin_write(IIC2_SDA, PIN_HIGH);//发送I2C总线结束信号
    rt_hw_us_delay(8);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC2_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    rt_pin_mode(IIC2_SDA, PIN_MODE_INPUT);      //SDA设置为输入
    rt_pin_write(IIC2_SDA, PIN_HIGH);
    rt_hw_us_delay(1);
    rt_pin_write(IIC2_SCL, PIN_HIGH);;
    rt_hw_us_delay(1);
    while(rt_pin_read(IIC2_SDA))
    {
        ucErrTime++;
    //  rt_hw_us_delay(4);
        if(ucErrTime>250)
        {
            IIC2_Stop();
            return 1;
        }
    }
    rt_pin_write(IIC2_SCL, PIN_LOW);//时钟输出0
    return 0;
}
//产生ACK应答
void IIC2_Ack(void)
{
    rt_pin_write(IIC2_SCL, PIN_LOW);
    rt_pin_mode(IIC2_SDA, PIN_MODE_OUTPUT);
    rt_pin_write(IIC2_SDA, PIN_LOW);
    rt_hw_us_delay(30);
    rt_pin_write(IIC2_SCL, PIN_HIGH);;
    rt_hw_us_delay(4);
    rt_pin_write(IIC2_SCL, PIN_LOW);
}
//不产生ACK应答
void IIC2_NAck(void)
{
    rt_pin_write(IIC2_SCL, PIN_LOW);
    rt_pin_mode(IIC2_SDA, PIN_MODE_OUTPUT);
    rt_pin_write(IIC2_SDA, PIN_HIGH);
    rt_hw_us_delay(10);
    rt_pin_write(IIC2_SCL, PIN_HIGH);;
    rt_hw_us_delay(10);
    rt_pin_write(IIC2_SCL, PIN_LOW);
}
//IIC2发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC2_Send_Byte(uint8_t txd)
{
    uint8_t t;
    rt_pin_mode(IIC2_SDA, PIN_MODE_OUTPUT);
    rt_pin_write(IIC2_SCL, PIN_LOW);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        //IIC2_SDA=(txd&0x80)>>7;
        rt_pin_write(IIC2_SDA, (txd&0x80)>>7);
        txd<<=1;
        rt_hw_us_delay(10);   //对TEA5767这三个延时都是必须的
        rt_pin_write(IIC2_SCL, PIN_HIGH);;
        rt_hw_us_delay(10);
        rt_pin_write(IIC2_SCL, PIN_LOW);
        rt_hw_us_delay(10);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC2_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    rt_pin_mode(IIC2_SDA, PIN_MODE_INPUT);//SDA设置为输入
    for(i=0;i<8;i++ )
    {
    rt_pin_write(IIC2_SCL, PIN_LOW);
    rt_hw_us_delay(10);
        rt_pin_write(IIC2_SCL, PIN_HIGH);;
        receive<<=1;
        if(rt_pin_read(IIC2_SDA))receive++;
        rt_hw_us_delay(10);
    }
    if (!ack)
        IIC2_NAck();//发送nACK
    else
        IIC2_Ack(); //发送ACK
    return receive;
}
