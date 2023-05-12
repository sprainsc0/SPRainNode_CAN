#ifndef CANARD_SCALAR_H_INCLUDED
#define CANARD_SCALAR_H_INCLUDED

#include "canard.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif
  
#ifndef CANARD_ASSERT
// Intentional violation of MISRA: assertion macro cannot be replaced with a function definition.
#    define CANARD_ASSERT(x) assert(x)  // NOSONAR
#endif
/**
 * This function can be used to extract values from received UAVCAN transfers. It decodes a scalar value -
 * boolean, integer, character, or floating point - from the specified bit position in the RX transfer buffer.
 * Simple single-frame transfers can also be parsed manually.
 *
 * Returns the number of bits successfully decoded, which may be less than requested if operation ran out of
 * buffer boundaries, or negated error code, such as invalid argument.
 *
 * Caveat:  This function works correctly only on platforms that use two's complement signed integer representation.
 *          I am not aware of any modern microarchitecture that uses anything else than two's complement, so it should
 *          not affect portability in any way.
 *
 * The type of value pointed to by 'out_value' is defined as follows:
 *
 *  | bit_length | value_is_signed | out_value points to                      |
 *  |------------|-----------------|------------------------------------------|
 *  | 1          | false           | bool (may be incompatible with uint8_t!) |
 *  | 1          | true            | N/A                                      |
 *  | [2, 8]     | false           | uint8_t, or char                         |
 *  | [2, 8]     | true            | int8_t, or char                          |
 *  | [9, 16]    | false           | uint16_t                                 |
 *  | [9, 16]    | true            | int16_t                                  |
 *  | [17, 32]   | false           | uint32_t                                 |
 *  | [17, 32]   | true            | int32_t, or 32-bit float                 |
 *  | [33, 64]   | false           | uint64_t                                 |
 *  | [33, 64]   | true            | int64_t, or 64-bit float                 |
 */
int16_t canardDecodeScalar(const CanardTransfer* transfer,    ///< The RX transfer where the data will be copied from
                           uint32_t bit_offset,                 ///< Offset, in bits, from the beginning of the transfer
                           uint8_t bit_length,                  ///< Length of the value, in bits; see the table
                           bool value_is_signed,                ///< True if the value can be negative; see the table
                           void* out_value);                    ///< Pointer to the output storage; see the table

/**
 * This function can be used to encode values for later transmission in a UAVCAN transfer. It encodes a scalar value -
 * boolean, integer, character, or floating point - and puts it to the specified bit position in the specified
 * contiguous buffer.
 * Simple single-frame transfers can also be encoded manually.
 *
 * Caveat:  This function works correctly only on platforms that use two's complement signed integer representation.
 *          I am not aware of any modern microarchitecture that uses anything else than two's complement, so it should
 *          not affect portability in any way.
 *
 * The type of value pointed to by 'value' is defined as follows:
 *
 *  | bit_length | value points to                          |
 *  |------------|------------------------------------------|
 *  | 1          | bool (may be incompatible with uint8_t!) |
 *  | [2, 8]     | uint8_t, int8_t, or char                 |
 *  | [9, 16]    | uint16_t, int16_t                        |
 *  | [17, 32]   | uint32_t, int32_t, or 32-bit float       |
 *  | [33, 64]   | uint64_t, int64_t, or 64-bit float       |
 */
void canardEncodeScalar(void* destination,      ///< Destination buffer where the result will be stored
                        uint32_t bit_offset,    ///< Offset, in bits, from the beginning of the destination buffer
                        uint8_t bit_length,     ///< Length of the value, in bits; see the table
                        const void* value);     ///< Pointer to the value; see the table

/**
 * Float16 marshaling helpers.
 * These functions convert between the native float and 16-bit float.
 * It is assumed that the native float is IEEE 754 single precision float, otherwise results will be unpredictable.
 * Vast majority of modern computers and microcontrollers use IEEE 754, so this limitation should not affect
 * portability.
 */
uint16_t canardConvertNativeFloatToFloat16(float value);
float canardConvertFloat16ToNativeFloat(uint16_t value);

#ifdef __cplusplus
}
#endif
#endif  // CANARD_DSDL_H_INCLUDED
