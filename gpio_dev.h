/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-23     Administrator       the first version
 */

#include <board.h>
#include <rtdevice.h>

#ifndef APPLICATIONS_GPIO_DEV_H_
#define APPLICATIONS_GPIO_DEV_H_
#define Bb_Valve1  GET_PIN(F,1)
//#define Bb_Valve2  GET_PIN(B,0)
//#define Bb_Valve3  GET_PIN(A,5)
//#define Bb_Valve4  GET_PIN(E,1)
#define Fan        GET_PIN(E,0)


void gpio_dev_init(void);

#endif /* APPLICATIONS_GPIO_DEV_H_ */
