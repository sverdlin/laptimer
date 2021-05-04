/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _POWER_H_
#define _POWER_H_

#include <avr/io.h>
#include "misc.h"

#define power_ac_disable()	\
do {				\
	ACSR = BIT(ACD);	\
} while (0)

#endif /* _POWER_H_ */
