/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _RECORD_H_
#define _RECORD_H_

#include <stdint.h>

#define ID_COUNT	16

struct lap {
	__uint24 ts;
	uint8_t id;
} __attribute__((packed));

#endif /* _RECORD_H_ */
