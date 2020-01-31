#include <stdint.h>

#include "crc.h"

/** Reflect bits in data byte. */
#define REFLECT_DATA(x)		((uint8_t) reflect(x, 8))

/** Reflect bits in CRC. */
#define REFLECT_CRC(x)		(reflect(x, CRC_SIZE))

#if (CRC_DATA_REFLECTED != 0) || (CRC_OUTPUT_REFLECTED != 0)
/**
 * Reflect bits in data given.
 *
 * @param data				Data to be reflected.
 * @param bits				Number of data bits.
 *
 * @return					Reflected data.
 */
static uint32_t reflect(uint32_t data, int32_t bits);
#endif

void crc_lookup_calc(uint32_t *tab, uint32_t poly)
{
	uint32_t remainder;
	uint32_t dividend;
	int32_t i;

	for (dividend = 0; dividend < 256; dividend++)
	{
		remainder = dividend << (CRC_SIZE - 8);

		for (i = 0; i < 8; i++)
		{
			if (remainder & (1 << (CRC_SIZE - 1)))
			{
				remainder = (remainder << 1) ^ poly;
			}
			else
			{
				remainder = remainder << 1;
			}
		}

#if CRC_SIZE == 32
		tab[dividend] = remainder;
#else
		tab[dividend] = remainder & ((1 << CRC_SIZE) - 1);
#endif
	}
}

uint32_t crc_calc(uint32_t *lookup, uint32_t seed, uint8_t *data, int32_t len)
{
	uint32_t remainder = 0;
	int32_t i;
	uint8_t byte;

	remainder = seed;

	for (i = 0; i < len; i++)
	{
#if CRC_DATA_REFLECTED != 0
		byte = REFLECT_DATA(data[i]);
#else
		byte = data[i];
#endif
		byte = byte ^ (remainder >> (CRC_SIZE - 8));
		remainder = lookup[byte] ^ (remainder << 8);
	}

#if CRC_OUTPUT_REFLECTED != 0
	return REFLECT_CRC(remainder);
#else
	return remainder;
#endif
}

uint32_t crc_calc_indirect(uint32_t *lookup, uint32_t seed, uint8_t *data, int32_t len)
{
	uint32_t remainder = seed;
	int32_t i;
	uint8_t byte;

	for (i = 0; i < len; i++)
	{
#if CRC_DATA_REFLECTED != 0
		byte = REFLECT_DATA(data[i]);
#else
		byte = data[i];
#endif
		remainder = ((remainder << 8) | byte) ^ lookup[(remainder >> (CRC_SIZE - 8)) & 0xFF];
	}

#if CRC_OUTPUT_REFLECTED != 0
	return REFLECT_CRC(remainder);
#else
	return remainder;
#endif
}

uint32_t seed_direct2indirect(uint32_t poly, uint32_t direct)
{
	int32_t i;
	uint32_t reg = direct;

	for (i = 0; i < CRC_SIZE; i++)
	{
		if (1 == (reg & 0x00000001))
		{
			reg = reg ^ poly;
			reg >>= 1;
			reg |= 0x80000000;
		}
		else
		{
			reg >>= 1;
		}
	}

	return reg;
}

#if (CRC_DATA_REFLECTED != 0) || (CRC_OUTPUT_REFLECTED != 0)
static uint32_t reflect(uint32_t data, int32_t bits)
{
	uint32_t retval = 0;
	int32_t i;

	for (i = 0; i < bits; i++)
	{
		if (data & (1 << i))
		{
			retval |= (1 << (bits - 1 - i));
		}
	}

	return retval;
}
#endif
