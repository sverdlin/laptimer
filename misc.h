/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _MISC_H_
#define _MISC_H_

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define BIT(n) (1 << (n))

#define max(a,b)			\
({					\
	__typeof__ (a) _a = (a);	\
	__typeof__ (b) _b = (b);	\
	_a > _b ? _a : _b;		\
})
#define min(a,b)			\
({					\
	__typeof__ (a) _a = (a);	\
	__typeof__ (b) _b = (b);	\
	_a < _b ? _a : _b;		\
})

#define BUILD_BUG_ON_ZERO(e) ((int)(sizeof(struct { int:(-!!(e)); })))

/* Force a compilation error if a constant expression is not a power of 2 */
#define BUILD_BUG_ON_NOT_POWER_OF_2(n)	\
	BUILD_BUG_ON_ZERO(((n) & ((n) - 1)) != 0)

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#else
#define cpu_to_le16(x) __builtin_bswap16(x)
#define cpu_to_le32(x) __builtin_bswap32(x)
#endif
#define le16_to_cpu(x) cpu_to_le16(x)
#define le32_to_cpu(x) cpu_to_le32(x)

#define likely(x)	__builtin_expect (!!(x), 1)
#define unlikely(x)	__builtin_expect (!!(x), 0)

#define __maybe_unused	__attribute__((unused))
#define __always_inline	inline __attribute__((__always_inline__))

#define barrier() __asm__ __volatile__("": : :"memory")

#endif /* _MISC_H_ */
