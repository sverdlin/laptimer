/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 */

#ifndef _HW_SENDER_H_
#define _HW_SENDER_H_

#include "gpio.h"

#define F_CPU		8000000

#define GPIO_PIN_CS	GPIO_PIN_B0
#define GPIO_PIN_CE	GPIO_PIN_D5
#define GPIO_PIN_RIRQ	GPIO_PIN_D6
#define GPIO_PIN_AD0	GPIO_PIN_D0
#define GPIO_PIN_AD1	GPIO_PIN_D1
#define GPIO_PIN_AD2	GPIO_PIN_D2
#define GPIO_PIN_AD3	GPIO_PIN_D3
#define GPIO_PIN_LEDG	GPIO_PIN_B6
#define GPIO_PIN_LEDY	GPIO_PIN_B5
#define GPIO_PIN_SIGNAL	GPIO_PIN_B4

#endif /* _HW_SENDER_H_ */
