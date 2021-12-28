/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-23     Administrator       the first version
 */

#include "gpio_dev.h"
void gpio_dev_init(void)
{
    rt_pin_mode(Fan, PIN_MODE_OUTPUT);
    rt_pin_mode(Bb_Valve1, PIN_MODE_OUTPUT);
//    rt_pin_mode(Bb_Valve2, PIN_MODE_OUTPUT);
//    rt_pin_mode(Bb_Valve3, PIN_MODE_OUTPUT);
//    rt_pin_mode(Bb_Valve4, PIN_MODE_OUTPUT);
    rt_pin_write(Fan, PIN_HIGH);
    rt_pin_write(Bb_Valve1, PIN_HIGH);
//    rt_pin_write(Bb_Valve2, PIN_HIGH);
    rt_thread_mdelay(500);
    rt_pin_write(Fan,PIN_LOW);
    rt_pin_write(Bb_Valve1, PIN_LOW);
//    rt_pin_write(Bb_Valve2, PIN_LOW);
//    rt_pin_write(Bb_Valve3, PIN_LOW);
//    rt_pin_write(Bb_Valve4, PIN_LOW);

}
