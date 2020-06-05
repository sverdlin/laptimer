/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 */

#ifndef _TIMER_H_
#define _TIMER_H_

#include "misc.h"

#define TMR0PRE(freq) ((F_CPU / (freq) <= 256) ? 1 : \
		       (F_CPU / 8 / (freq) <= 256) ? 8 : \
		       (F_CPU / 64 / (freq) <= 256) ? 64 : \
		       (F_CPU / 256 / (freq) <= 256) ? 256 : 1024)

#define TMR0PREBITS(freq) ((TMR0PRE(freq) == 1) ? BIT(CS00) : \
			   (TMR0PRE(freq) == 8) ? BIT(CS01) : \
			   (TMR0PRE(freq) == 64) ? BIT(CS00) | BIT(CS01) : \
			   (TMR0PRE(freq) == 256) ? BIT(CS02) : BIT(CS02) | BIT(CS00))

#define TMR0OCR(freq) (F_CPU / TMR0PRE(freq) / freq - 1 + \
		       BUILD_BUG_ON_ZERO(F_CPU / TMR0PRE(freq) / freq * freq * TMR0PRE(freq) != F_CPU))

#endif /* _TIMER_H_ */
