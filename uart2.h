/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-09     Administrator       the first version
 */
#ifndef APPLICATIONS_UART2_H_
#define APPLICATIONS_UART2_H_
#include <rtthread.h>
#include "ab32vg1_hal.h"

void uart2_init(void);
void uart2_send_val(unsigned char str[],uint8_t lengh);
void command_proc_init(void);

#endif /* APPLICATIONS_UART2_H_ */
