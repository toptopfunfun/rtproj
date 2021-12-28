/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-28     Administrator       the first version
 */



#ifndef __MYIIC_H
#define __MYIIC_H
//#include "sys.h"
//#include "pin.h"
#define IIC_SCL    GET_PIN(E,3)//SCL
#define IIC_SDA    GET_PIN(E,2)//SDA
//#define IIC_SCL    GET_PIN(E,6)//SCL
//#define IIC_SDA    GET_PIN(E,7)//SDA


#include <board.h>
#include <rtdevice.h>



//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);               //发送IIC开始信号
void IIC_Stop(void);                //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);         //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void);              //IIC等待ACK信号
void IIC_Ack(void);                 //IIC发送ACK信号
void IIC_NAck(void);                //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);
#endif
















