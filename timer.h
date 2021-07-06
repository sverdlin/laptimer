/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>
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
		       BUILD_BUG_ON_ZERO((F_CPU / TMR0PRE(freq) / freq) * freq * TMR0PRE(freq) != F_CPU))

#define TMR1PRE(per) ((per <= 65536) ? 1 : \
		      (per / 8 <= 65536) ? 8 : \
		      (per / 64 <= 65536) ? 64 : \
		      (per / 256 <= 65536) ? 256 : 1024)

#define TMR1PREBITS(per) ((TMR1PRE(per) == 1) ? BIT(CS10) : \
			  (TMR1PRE(per) == 8) ? BIT(CS11) : \
			  (TMR1PRE(per) == 64) ? BIT(CS10) | BIT(CS11) : \
			  (TMR1PRE(per) == 256) ? BIT(CS12) : BIT(CS12) | BIT(CS10))

#define TMR1OCR(per) (per / TMR1PRE(per) - 1)

#endif /* _TIMER_H_ */
