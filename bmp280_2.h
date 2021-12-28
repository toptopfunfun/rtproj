/*
 * Copyright (c) 2006-2021, JiaDun Data Development Topfun
 *
 * BMP2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-7       陶峰           用myiic驱动
 */
#ifndef APPLICATIONS_BMP280_2_H_
#define APPLICATIONS_BMP280_2_H_
#include <rtthread.h>
#include "ab32vg1_hal.h"
extern uint8_t line2sensor_num;




typedef         long signed int             BMP280_S32_t;
typedef         long unsigned int           BMP280_U32_t;
typedef         long long signed int        BMP280_S64_t;

#define dig_21_T1          bmp280_2_1.T1
#define dig_21_T2          bmp280_2_1.T2
#define dig_21_T3          bmp280_2_1.T3
#define dig_21_P1          bmp280_2_1.P1
#define dig_21_P2          bmp280_2_1.P2
#define dig_21_P3          bmp280_2_1.P3
#define dig_21_P4          bmp280_2_1.P4
#define dig_21_P5          bmp280_2_1.P5
#define dig_21_P6          bmp280_2_1.P6
#define dig_21_P7          bmp280_2_1.P7
#define dig_21_P8          bmp280_2_1.P8
#define dig_21_P9          bmp280_2_1.P9

#define dig_22_T1          bmp280_2_2.T1
#define dig_22_T2          bmp280_2_2.T2
#define dig_22_T3          bmp280_2_2.T3
#define dig_22_P1          bmp280_2_2.P1
#define dig_22_P2          bmp280_2_2.P2
#define dig_22_P3          bmp280_2_2.P3
#define dig_22_P4          bmp280_2_2.P4
#define dig_22_P5          bmp280_2_2.P5
#define dig_22_P6          bmp280_2_2.P6
#define dig_22_P7          bmp280_2_2.P7
#define dig_22_P8          bmp280_2_2.P8
#define dig_22_P9          bmp280_2_2.P9








typedef struct
{
    /* T1~P9 为补偿系数 */
    uint16_t T1;
    int16_t T2;
    int16_t T3;
    uint16_t P1;
    int16_t P2;
    int16_t P3;
    int16_t P4;
    int16_t P5;
    int16_t P6;
    int16_t P7;
    int16_t P8;
    int16_t P9;
} BMP280_2;
extern  BMP280_2 bmp280_2_1,bmp280_2_2;





void Bmp2_Init(uint8_t bmpadd);
uint8_t BMP280_2_ReadID(uint8_t bmpadd);
uint8_t  BMP280_2_GetStatus(uint8_t bmpadd,uint8_t status_flag);
double BMP280_2_Get_Pressure(uint8_t bmpadd);
double BMP280_2_Get_Temperature(uint8_t bmpadd);



 uint8_t BMP280_2_Read_Byte(uint8_t bmpadd,uint8_t reg);
 void BMP280_2_Write_Byte(uint8_t bmpadd,uint8_t reg,uint8_t data);
 int bmpline2_proc(void);

#endif /* APPLICATIONS_BMP280_H_ */
