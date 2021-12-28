/*
 * Copyright (c) 2006-2021, JiaDun Data Development Topfun
 *
 * BMP2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-7       陶峰           用myiic驱动
 */
#include "bmp280.h"
#include"myiic.h"
#include <uart2.h>
#include "stdio.h"
#include "string.h"
uint8_t line1sensor_num;
  uint8_t BMP280_Read_Byte(uint8_t bmpadd,uint8_t reg)
 {
     uint8_t rec_data;
     IIC_Start();
     IIC_Send_Byte(bmpadd<<1|0);
     IIC_Wait_Ack();
     IIC_Send_Byte(reg);
     IIC_Wait_Ack();

     IIC_Start();
     IIC_Send_Byte(bmpadd<<1|1);
     IIC_Wait_Ack();
     rec_data = IIC_Read_Byte(0);    //不应答
     IIC_Stop();
     return rec_data;
 }

  void BMP280_Write_Byte(uint8_t bmpadd,uint8_t reg,uint8_t data)
 {
     IIC_Start();
     IIC_Send_Byte(bmpadd<<1);
     IIC_Wait_Ack();
     IIC_Send_Byte(reg);
     IIC_Wait_Ack();

     IIC_Send_Byte(data);
     IIC_Wait_Ack();
     IIC_Stop();
 }


uint8_t BMP280_ReadID(uint8_t bmpadd)
{
    return BMP280_Read_Byte(bmpadd,BMP280_CHIPID_REG);
}


void BMP280_Set_TemOversamp(uint8_t bmpadd,BMP_OVERSAMPLE_MODE * Oversample_Mode)
{
    uint8_t Regtmp;

    Regtmp = ((Oversample_Mode->T_Osample)<<5)|
             ((Oversample_Mode->P_Osample)<<2)|
             ((Oversample_Mode)->WORKMODE);
    if(bmpadd==0x76)
    {
        BMP280_Write_Byte(BMP280_ADDRESS,BMP280_CTRLMEAS_REG,Regtmp);
    }
    else
    {
        BMP280_Write_Byte(BMP280_ADDRESS+1,BMP280_CTRLMEAS_REG,Regtmp);
    }

}


//设置保持时间和滤波器分频因子
void BMP280_Set_Standby_FILTER(uint8_t bmpadd,BMP_CONFIG * BMP_Config)
{
    uint8_t Regtmp;
    Regtmp = ((BMP_Config->T_SB)<<5)|
             ((BMP_Config->FILTER_COEFFICIENT)<<2)|
             ((BMP_Config->SPI_EN));
    if(bmpadd==0x76)
        {
            BMP280_Write_Byte(BMP280_ADDRESS,BMP280_CONFIG_REG,Regtmp);
        }
    else
    {
        BMP280_Write_Byte(BMP280_ADDRESS+1,BMP280_CONFIG_REG,Regtmp);
    }

}





BMP280  bmp280_1,bmp280_2;     //用来保存存在芯片内ROM补偿参数
void Bmp1_Init(uint8_t bmpadd)
{
    uint8_t Lsb,Msb,bmp280id;
    line1sensor_num=0;
    bmp280id=0;
    bmp280id=BMP280_ReadID(BMP280_ADDRESS);
    if (bmp280id==0x58) line1sensor_num=line1sensor_num+1;
    bmp280id=0;
    bmp280id=BMP280_ReadID(BMP280_ADDRESS+1);
    if (bmp280id==0x58) line1sensor_num=line1sensor_num+1;
    rt_kprintf("\r\nLine1 Sensor Number is:%d ",line1sensor_num);
    if (line1sensor_num>=1)
    {

        //温度传感器的矫正值
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_T1_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_T1_MSB_REG);
        bmp280_1.T1 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_T2_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_T2_MSB_REG);
        bmp280_1.T2 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_T3_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_T3_MSB_REG);
        bmp280_1.T3 = (((uint16_t)Msb)<<8) + Lsb;

        //大气压传感器的矫正值
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P1_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P1_MSB_REG);
        bmp280_1.P1 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P2_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P2_MSB_REG);
        bmp280_1.P2 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P3_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P3_MSB_REG);
        bmp280_1.P3 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P4_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P4_MSB_REG);
        bmp280_1.P4 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P5_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P5_MSB_REG);
        bmp280_1.P5 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P6_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P6_MSB_REG);
        bmp280_1.P6 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P7_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P7_MSB_REG);
        bmp280_1.P7 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P8_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P8_MSB_REG);
        bmp280_1.P8 = (((uint16_t)Msb)<<8) + Lsb;
        Lsb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P9_LSB_REG);
        Msb = BMP280_Read_Byte(bmpadd,BMP280_DIG_P9_MSB_REG);
        bmp280_1.P9 = (((uint16_t)Msb)<<8) + Lsb;

        BMP280_Write_Byte(bmpadd,BMP280_RESET_REG,BMP280_RESET_VALUE); //往复位寄存器写入给定值

        BMP_OVERSAMPLE_MODE         BMP_OVERSAMPLE_MODEStructure;
        BMP_OVERSAMPLE_MODEStructure.P_Osample = BMP280_P_MODE_3;
        BMP_OVERSAMPLE_MODEStructure.T_Osample = BMP280_T_MODE_1;
        BMP_OVERSAMPLE_MODEStructure.WORKMODE  = BMP280_NORMAL_MODE;
        BMP280_Set_TemOversamp(bmpadd,&BMP_OVERSAMPLE_MODEStructure);

        BMP_CONFIG                  BMP_CONFIGStructure;
        BMP_CONFIGStructure.T_SB = BMP280_T_SB1;
        BMP_CONFIGStructure.FILTER_COEFFICIENT = BMP280_FILTER_MODE_4;
        BMP_CONFIGStructure.SPI_EN = 0;

        BMP280_Set_Standby_FILTER(bmpadd,&BMP_CONFIGStructure);
        if (line1sensor_num==2)
        {
            /********************接下来读出同一总线上第二个传感器矫正参数*********************/
            //温度传感器的矫正值
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_T1_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_T1_MSB_REG);
            bmp280_2.T1 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_T2_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_T2_MSB_REG);
            bmp280_2.T2 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_T3_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_T3_MSB_REG);
            bmp280_2.T3 = (((uint16_t)Msb)<<8) + Lsb;

            //大气压传感器的矫正值
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P1_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P1_MSB_REG);
            bmp280_2.P1 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P2_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P2_MSB_REG);
            bmp280_2.P2 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P3_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P3_MSB_REG);
            bmp280_2.P3 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P4_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P4_MSB_REG);
            bmp280_2.P4 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P5_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P5_MSB_REG);
            bmp280_2.P5 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P6_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P6_MSB_REG);
            bmp280_2.P6 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P7_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P7_MSB_REG);
            bmp280_2.P7 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P8_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P8_MSB_REG);
            bmp280_2.P8 = (((uint16_t)Msb)<<8) + Lsb;
            Lsb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P9_LSB_REG);
            Msb = BMP280_Read_Byte(bmpadd+1,BMP280_DIG_P9_MSB_REG);
            bmp280_2.P9 = (((uint16_t)Msb)<<8) + Lsb;

            BMP280_Write_Byte(BMP280_ADDRESS+1,BMP280_RESET_REG,BMP280_RESET_VALUE);

            BMP_OVERSAMPLE_MODEStructure.P_Osample = BMP280_P_MODE_3;
            BMP_OVERSAMPLE_MODEStructure.T_Osample = BMP280_T_MODE_1;
            BMP_OVERSAMPLE_MODEStructure.WORKMODE  = BMP280_NORMAL_MODE;
            BMP280_Set_TemOversamp(bmpadd+1,&BMP_OVERSAMPLE_MODEStructure);


            BMP_CONFIGStructure.T_SB = BMP280_T_SB1;
            BMP_CONFIGStructure.FILTER_COEFFICIENT = BMP280_FILTER_MODE_4;
            BMP_CONFIGStructure.SPI_EN = 0;

            BMP280_Set_Standby_FILTER(bmpadd+1,&BMP_CONFIGStructure);
        }
    }
}
//获取BMP当前状态

uint8_t  BMP280_GetStatus(uint8_t bmpadd,uint8_t status_flag)
{
    uint8_t flag;
    flag = BMP280_Read_Byte(bmpadd,BMP280_STATUS_REG);
    if(flag&status_flag)    return 1;
    else return 0;
}


BMP280_S32_t t_fine;            //用于计算补偿

BMP280_S32_t bmp280_compensate_T_int32(uint8_t bmpadd,BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2, T;
if(bmpadd==0x76)
{

    var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig1_T1<<1))) * ((BMP280_S32_t)dig1_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig1_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig1_T1))) >> 12) *
    ((BMP280_S32_t)dig1_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}
else
{

    var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig2_T1<<1))) * ((BMP280_S32_t)dig2_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig2_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig2_T1))) >> 12) *
    ((BMP280_S32_t)dig2_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}
}


BMP280_U32_t bmp280_compensate_P_int64(uint8_t bmpadd,BMP280_S32_t adc_P)
{
    BMP280_S64_t var1, var2, p;
if (bmpadd==0x76)
{
    var1 = ((BMP280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BMP280_S64_t)dig1_P6;
    var2 = var2 + ((var1*(BMP280_S64_t)dig1_P5)<<17);
    var2 = var2 + (((BMP280_S64_t)dig1_P4)<<35);
    var1 = ((var1 * var1 * (BMP280_S64_t)dig1_P3)>>8) + ((var1 * (BMP280_S64_t)dig1_P2)<<12);
    var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig1_P1)>>33;
    if (var1 == 0)
    {
    return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BMP280_S64_t)dig1_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BMP280_S64_t)dig1_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig1_P7)<<4);
    return (BMP280_U32_t)p;
}
else {
    var1 = ((BMP280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BMP280_S64_t)dig2_P6;
    var2 = var2 + ((var1*(BMP280_S64_t)dig2_P5)<<17);
    var2 = var2 + (((BMP280_S64_t)dig2_P4)<<35);
    var1 = ((var1 * var1 * (BMP280_S64_t)dig2_P3)>>8) + ((var1 * (BMP280_S64_t)dig2_P2)<<12);
    var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig2_P1)>>33;
    if (var1 == 0)
    {
    return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BMP280_S64_t)dig2_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BMP280_S64_t)dig2_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig2_P7)<<4);
    return (BMP280_U32_t)p;

}


}

//大气压值-Pa
double BMP280_Get_Pressure(uint8_t bmpadd)
{
    uint8_t XLsb,Lsb, Msb;
    long signed Bit32;
    double pressure;
    XLsb = BMP280_Read_Byte(bmpadd,BMP280_PRESSURE_XLSB_REG);
    Lsb  = BMP280_Read_Byte(bmpadd,BMP280_PRESSURE_LSB_REG);
    Msb  = BMP280_Read_Byte(bmpadd,BMP280_PRESSURE_MSB_REG);
    Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);   //寄存器的值,组成一个浮点数
    pressure = bmp280_compensate_P_int64(bmpadd,Bit32);
    return pressure;
}

//温度值-℃
double BMP280_Get_Temperature(uint8_t bmpadd)
{
    uint8_t XLsb,Lsb, Msb;
    long signed Bit32;
    double temperature;
    XLsb = BMP280_Read_Byte(bmpadd,BMP280_TEMPERATURE_XLSB_REG);
    Lsb  = BMP280_Read_Byte(bmpadd,BMP280_TEMPERATURE_LSB_REG);
    Msb  = BMP280_Read_Byte(bmpadd,BMP280_TEMPERATURE_MSB_REG);
    Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);   //寄存器的值,组成一个浮点数
    temperature = bmp280_compensate_T_int32(bmpadd,Bit32);
    return temperature;
}
static void read_bmbl1_entry(void *parameter)
{

    u_int8_t i;
    char  temp_buff2[10];
    double  BMP_Temperature;
    double  BMP_Pressure;
    int32_t temper,press;


    unsigned char bkcmd[]={0x62,0x6B,0x63,0x6D,0x64,0x3D,0x30};//bkcmd=0
    unsigned char scrcomm_init[]={0x00,0xff,0xff,0xff};//串口屏先把乱数据清掉
    unsigned char scrcomm_backadd[]={0xff,0xff,0xff};
    unsigned char temp_toscr2[] = {0x78,0x30,0x2e,0x76,0x61,0x6c,0x3d,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
        //x     0   .    v     a    l    =
    unsigned char baro_toscr2[] = {0x6e,0x30,0x2e,0x76,0x61,0x6c,0x3d,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    //6E 30 2E 76 61 6C 3D 32 30 FF FF FF 6E 31 2E 76 61 6C 3D 38 30 37 37 38 FF FF FF
    //n  0

    uart2_send_val(bkcmd,7);//关闭屏幕回显，必须包含下面这名后缀
    uart2_send_val(scrcomm_backadd,3);
    uart2_send_val(scrcomm_init,4);//每次向串口屏发数据，先清一下乱数据



    while (1)
    {
        if (line1sensor_num>=1)
        {
            while(BMP280_GetStatus(BMP280_ADDRESS,BMP280_MEASURING) != 0);
            while(BMP280_GetStatus(BMP280_ADDRESS,BMP280_IM_UPDATE) != 0);
            BMP_Temperature = BMP280_Get_Temperature(BMP280_ADDRESS);
            temper=BMP_Temperature/10;
            BMP_Pressure = BMP280_Get_Pressure(BMP280_ADDRESS);
            press=BMP_Pressure/256;
            sprintf(temp_buff2,"%d",temper);//将温度值转为字符串
            for(i=0;i<strlen(temp_buff2);i++)
            {
                temp_toscr2[7+i] =temp_buff2[i];
            }
            temp_toscr2[1]=0x30;
            uart2_send_val(temp_toscr2,strlen(temp_buff2)+7);
            uart2_send_val(scrcomm_backadd,3);


            sprintf(temp_buff2,"%d", press);//将气压值转为字符串
                        for(i=0;i<strlen(temp_buff2);i++)
                        {
                            baro_toscr2[7+i] =temp_buff2[i];
                        }
            baro_toscr2[1]=0x30;
            uart2_send_val( baro_toscr2,strlen(temp_buff2)+7);
            uart2_send_val(scrcomm_backadd,3);

            if (line1sensor_num==2)
            {
                    while(BMP280_GetStatus(BMP280_ADDRESS+1,BMP280_MEASURING) != 0);
                    while(BMP280_GetStatus(BMP280_ADDRESS+1,BMP280_IM_UPDATE) != 0);
                    BMP_Temperature = BMP280_Get_Temperature(BMP280_ADDRESS+1);
                    temper=BMP_Temperature/10;
                    BMP_Pressure = BMP280_Get_Pressure(BMP280_ADDRESS+1);
                    press=BMP_Pressure/256;

                    sprintf(temp_buff2,"%d",temper);//将温度值转为字符串
                    for(i=0;i<strlen(temp_buff2);i++)
                    {
                        temp_toscr2[7+i] =temp_buff2[i];
                    }
                    temp_toscr2[1]=0x31;
                    uart2_send_val(temp_toscr2,strlen(temp_buff2)+7);
                    uart2_send_val(scrcomm_backadd,3);


                    sprintf(temp_buff2,"%d", press);//将气压值转为字符串
                    for(i=0;i<strlen(temp_buff2);i++)
                    {
                        baro_toscr2[7+i] =temp_buff2[i];
                    }
                    baro_toscr2[1]=0x31;
                    uart2_send_val( baro_toscr2,strlen(temp_buff2)+7);
                    uart2_send_val(scrcomm_backadd,3);
            }
        }

        rt_thread_delay(500);
    }
}

int bmpline1_proc(void)
{
   rt_thread_t bmpl1_thread;

   bmpl1_thread = rt_thread_create(
           "bmpl1",
           read_bmbl1_entry,
           RT_NULL,
           1024,
           RT_THREAD_PRIORITY_MAX/2,
           20);
   if (bmpl1_thread != RT_NULL)
   {
       rt_thread_startup(bmpl1_thread);
   }

   return RT_EOK;
}
