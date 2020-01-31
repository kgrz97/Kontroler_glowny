#ifndef CRC_H_
#define CRC_H_

/** Size of CRC checksum. */
#define CRC_SIZE				32

/** Is CRC input data reflected. */
#define CRC_DATA_REFLECTED		0

/** Is CRC output data reflected. */
#define CRC_OUTPUT_REFLECTED	0

/**
 * Create lookup table for CRC calculation with a given polynomial.
 *
 * @param tab				Table to be filled with data.
 * @param poly				CRC polynomial.
 */
void crc_lookup_calc(uint32_t *tab, uint32_t poly);

/**
 * Calculate direct CRC from a given data buffer.
 *
 * @param lookup			Lookup table for polynomial used.
 * @param seed				CRC initial value.
 * @param data				Data buffer.
 * @param len				Length of the data buffer.
 *
 * @return					Calculated CRC value.
 */
uint32_t crc_calc(uint32_t *lookup, uint32_t seed, uint8_t *data, int32_t len);

/**
 * Calculate indirect CRC from a given data buffer.
 *
 * @param lookup			Lookup table for polynomial used.
 * @param seed				CRC initial value.
 * @param data				Data buffer.
 * @param len				Length of the data buffer.
 *
 * @return					Calculated CRC value.
 */
uint32_t crc_calc_indirect(uint32_t *lookup, uint32_t seed, uint8_t *data, int32_t len);

/**
 * Convert seed for direct CRC to indirect CRC seed.
 *
 * @param poly				CRC polynomial.
 * @param direct			Direct CRC seed.
 *
 * @return					Indirect CRC seed.
 */
uint32_t seed_direct2indirect(uint32_t poly, uint32_t direct);

#endif /* CRC_H_ */
