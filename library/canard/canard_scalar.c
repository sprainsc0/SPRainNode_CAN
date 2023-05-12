#include "canard_scalar.h"
#include <assert.h>
#include <float.h>
#include <string.h>

/// By default, this macro resolves to the standard assert(). The user can redefine this if necessary.
/// To disable assertion checks completely, make it expand into `(void)(0)`.
#ifndef CANARD_ASSERT
// Intentional violation of MISRA: assertion macro cannot be replaced with a function definition.
#    define CANARD_ASSERT(x) assert(x)  // NOSONAR
#endif

static void copyBitArray(const uint8_t* src, uint32_t src_offset, uint32_t src_len,
                        uint8_t* dst, uint32_t dst_offset)
{
    CANARD_ASSERT(src_len > 0U);

    // Normalizing inputs
    src += src_offset / 8U;
    dst += dst_offset / 8U;

    src_offset %= 8U;
    dst_offset %= 8U;

    const size_t last_bit = src_offset + src_len;
    while (last_bit - src_offset)
    {
        const uint8_t src_bit_offset = (uint8_t)(src_offset % 8U);
        const uint8_t dst_bit_offset = (uint8_t)(dst_offset % 8U);

        const uint8_t max_offset = src_bit_offset > dst_bit_offset ? src_bit_offset : dst_bit_offset;
        const uint32_t copy_bits = (last_bit - src_offset) < (8U - max_offset) ? (last_bit - src_offset) : (8U - max_offset);

        const uint8_t write_mask = (uint8_t)((uint8_t)(0xFF00U >> copy_bits) >> dst_bit_offset);
        const uint8_t src_data = (uint8_t)(((uint32_t)src[src_offset / 8U] << src_bit_offset) >> dst_bit_offset);

        dst[dst_offset / 8U] =
            (uint8_t)(((uint32_t)dst[dst_offset / 8U] & (uint32_t)~write_mask) | (uint32_t)(src_data & write_mask));

        src_offset += copy_bits;
        dst_offset += copy_bits;
    }
}

static int16_t descatterTransferPayload(const CanardTransfer* transfer,
                                                 uint32_t bit_offset,
                                                 uint8_t bit_length,
                                                 void* output)
{
    CANARD_ASSERT(transfer != 0);

    if (bit_offset >= transfer->payload_size * 8)
    {
        return 0;       // Out of range, reading zero bits
    }

    if (bit_offset + bit_length > transfer->payload_size * 8)
    {
        bit_length = (uint8_t)(transfer->payload_size * 8U - bit_offset);
    }

    CANARD_ASSERT(bit_length > 0);

    copyBitArray((const uint8_t *)(transfer->payload), bit_offset, bit_length, (uint8_t*) output, 0);

    return bit_length;
}

static bool isBigEndian(void)
{
#if defined(BYTE_ORDER) && defined(BIG_ENDIAN)
    return BYTE_ORDER == BIG_ENDIAN;                // Some compilers offer this neat shortcut
#else
    union
    {
        uint16_t a;
        uint8_t b[2];
    } u;
    u.a = 1;
    return u.b[1] == 1;                             // Some don't...
#endif
}

static void swapByteOrder(void* data, size_t size)
{
    CANARD_ASSERT(data != NULL);

    uint8_t* const bytes = (uint8_t*) data;

    size_t fwd = 0;
    size_t rev = size - 1;

    while (fwd < rev)
    {
        const uint8_t x = bytes[fwd];
        bytes[fwd] = bytes[rev];
        bytes[rev] = x;
        fwd++;
        rev--;
    }
}

int16_t canardDecodeScalar(const CanardTransfer* transfer,
                           uint32_t bit_offset,
                           uint8_t bit_length,
                           bool value_is_signed,
                           void* out_value)
{
    if (transfer == NULL || out_value == NULL)
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    if (bit_length < 1 || bit_length > 64)
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    if (bit_length == 1 && value_is_signed)
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    /*
     * Reading raw bytes into the temporary storage.
     * Luckily, C guarantees that every element is aligned at the beginning (lower address) of the union.
     */
    union
    {
        bool     boolean;       ///< sizeof(bool) is implementation-defined, so it has to be handled separately
        uint8_t  u8;            ///< Also char
        int8_t   s8;
        uint16_t u16;
        int16_t  s16;
        uint32_t u32;
        int32_t  s32;           ///< Also float, possibly double, possibly long double (depends on implementation)
        uint64_t u64;
        int64_t  s64;           ///< Also double, possibly float, possibly long double (depends on implementation)
        uint8_t bytes[8];
    } storage;

    memset(&storage, 0, sizeof(storage));   // This is important

    const int16_t result = descatterTransferPayload(transfer, bit_offset, bit_length, &storage.bytes[0]);
    if (result <= 0)
    {
        return result;
    }

    CANARD_ASSERT((result > 0) && (result <= 64) && (result <= bit_length));

    /*
     * The bit copy algorithm assumes that more significant bits have lower index, so we need to shift some.
     * Extra most significant bits will be filled with zeroes, which is fine.
     * Coverity Scan mistakenly believes that the array may be overrun if bit_length == 64; however, this branch will
     * not be taken if bit_length == 64, because 64 % 8 == 0.
     */
    if ((bit_length % 8) != 0)
    {
        // coverity[overrun-local]
        storage.bytes[bit_length / 8U] = (uint8_t)(storage.bytes[bit_length / 8U] >> ((8U - (bit_length % 8U)) & 7U));
    }

    /*
     * Determining the closest standard byte length - this will be needed for byte reordering and sign bit extension.
     */
    uint8_t std_byte_length = 0;
    if      (bit_length == 1)   { std_byte_length = sizeof(bool); }
    else if (bit_length <= 8)   { std_byte_length = 1; }
    else if (bit_length <= 16)  { std_byte_length = 2; }
    else if (bit_length <= 32)  { std_byte_length = 4; }
    else if (bit_length <= 64)  { std_byte_length = 8; }
    else
    {
        CANARD_ASSERT(false);
        return -CANARD_ERROR_INTERNAL;
    }

    CANARD_ASSERT((std_byte_length > 0) && (std_byte_length <= 8));

    /*
     * Flipping the byte order if needed.
     */
    if (isBigEndian())
    {
        swapByteOrder(&storage.bytes[0], std_byte_length);
    }

    /*
     * Extending the sign bit if needed. I miss templates.
     * Note that we operate on unsigned values in order to avoid undefined behaviors.
     */
    if (value_is_signed && (std_byte_length * 8 != bit_length))
    {
        if (bit_length <= 8)
        {
            if ((storage.u8 & (1U << (bit_length - 1U))) != 0)                           // If the sign bit is set...
            {
                storage.u8 |= (uint8_t) 0xFFU & (uint8_t) ~((1U << bit_length) - 1U);   // ...set all bits above it.
            }
        }
        else if (bit_length <= 16)
        {
            if ((storage.u16 & (1U << (bit_length - 1U))) != 0)
            {
                storage.u16 |= (uint16_t) 0xFFFFU & (uint16_t) ~((1U << bit_length) - 1U);
            }
        }
        else if (bit_length <= 32)
        {
            if ((storage.u32 & (((uint32_t) 1) << (bit_length - 1U))) != 0)
            {
                storage.u32 |= (uint32_t) 0xFFFFFFFFUL & (uint32_t) ~((((uint32_t) 1) << bit_length) - 1U);
            }
        }
        else if (bit_length < 64)   // Strictly less, this is not a typo
        {
            if ((storage.u64 & (((uint64_t) 1) << (bit_length - 1U))) != 0)
            {
                storage.u64 |= (uint64_t) 0xFFFFFFFFFFFFFFFFULL & (uint64_t) ~((((uint64_t) 1) << bit_length) - 1U);
            }
        }
        else
        {
            CANARD_ASSERT(false);
            return -CANARD_ERROR_INTERNAL;
        }
    }

    /*
     * Copying the result out.
     */
    if (value_is_signed)
    {
        if      (bit_length <= 8)   { *( (int8_t*) out_value) = storage.s8;  }
        else if (bit_length <= 16)  { *((int16_t*) out_value) = storage.s16; }
        else if (bit_length <= 32)  { *((int32_t*) out_value) = storage.s32; }
        else if (bit_length <= 64)  { *((int64_t*) out_value) = storage.s64; }
        else
        {
            CANARD_ASSERT(false);
            return -CANARD_ERROR_INTERNAL;
        }
    }
    else
    {
        if      (bit_length == 1)   { *(    (bool*) out_value) = storage.boolean; }
        else if (bit_length <= 8)   { *( (uint8_t*) out_value) = storage.u8;  }
        else if (bit_length <= 16)  { *((uint16_t*) out_value) = storage.u16; }
        else if (bit_length <= 32)  { *((uint32_t*) out_value) = storage.u32; }
        else if (bit_length <= 64)  { *((uint64_t*) out_value) = storage.u64; }
        else
        {
            CANARD_ASSERT(false);
            return -CANARD_ERROR_INTERNAL;
        }
    }

    CANARD_ASSERT(result <= bit_length);
    CANARD_ASSERT(result > 0);
    return result;
}

void canardEncodeScalar(void* destination,
                        uint32_t bit_offset,
                        uint8_t bit_length,
                        const void* value)
{
    /*
     * This function can only fail due to invalid arguments, so it was decided to make it return void,
     * and in the case of bad arguments try the best effort or just trigger an CANARD_ASSERTion failure.
     * Maybe not the best solution, but it simplifies the API.
     */
    CANARD_ASSERT(destination != NULL);
    CANARD_ASSERT(value != NULL);

    if (bit_length > 64)
    {
        CANARD_ASSERT(false);
        bit_length = 64;
    }

    if (bit_length < 1)
    {
        CANARD_ASSERT(false);
        bit_length = 1;
    }

    /*
     * Preparing the data in the temporary storage.
     */
    union
    {
        bool     boolean;
        uint8_t  u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
        uint8_t bytes[8];
    } storage;

    memset(&storage, 0, sizeof(storage));

    uint8_t std_byte_length = 0;

    // Extra most significant bits can be safely ignored here.
    if      (bit_length == 1)   { std_byte_length = sizeof(bool);   storage.boolean = (*((bool*) value) != 0); }
    else if (bit_length <= 8)   { std_byte_length = 1;              storage.u8  = *((uint8_t*) value);  }
    else if (bit_length <= 16)  { std_byte_length = 2;              storage.u16 = *((uint16_t*) value); }
    else if (bit_length <= 32)  { std_byte_length = 4;              storage.u32 = *((uint32_t*) value); }
    else if (bit_length <= 64)  { std_byte_length = 8;              storage.u64 = *((uint64_t*) value); }
    else
    {
        CANARD_ASSERT(false);
    }

    CANARD_ASSERT(std_byte_length > 0);

    if (isBigEndian())
    {
        swapByteOrder(&storage.bytes[0], std_byte_length);
    }

    /*
     * The bit copy algorithm assumes that more significant bits have lower index, so we need to shift some.
     * Extra least significant bits will be filled with zeroes, which is fine.
     * Extra most significant bits will be discarded here.
     * Coverity Scan mistakenly believes that the array may be overrun if bit_length == 64; however, this branch will
     * not be taken if bit_length == 64, because 64 % 8 == 0.
     */
    if ((bit_length % 8) != 0)
    {
        // coverity[overrun-local]
        storage.bytes[bit_length / 8U] = (uint8_t)(storage.bytes[bit_length / 8U] << ((8U - (bit_length % 8U)) & 7U));
    }

    /*
     * Now, the storage contains properly serialized scalar. Copying it out.
     */
    copyBitArray(&storage.bytes[0], 0, bit_length, (uint8_t*) destination, bit_offset);
}


uint16_t canardConvertNativeFloatToFloat16(float value)
{
    CANARD_ASSERT(sizeof(float) == 4);

    union FP32
    {
        uint32_t u;
        float f;
    };

    const union FP32 f32inf = { 255UL << 23U };
    const union FP32 f16inf = { 31UL << 23U };
    const union FP32 magic = { 15UL << 23U };
    const uint32_t sign_mask = 0x80000000UL;
    const uint32_t round_mask = ~0xFFFUL;

    union FP32 in;
    in.f = value;
    uint32_t sign = in.u & sign_mask;
    in.u ^= sign;

    uint16_t out = 0;

    if (in.u >= f32inf.u)
    {
        out = (in.u > f32inf.u) ? (uint16_t)0x7FFFU : (uint16_t)0x7C00U;
    }
    else
    {
        in.u &= round_mask;
        in.f *= magic.f;
        in.u -= round_mask;
        if (in.u > f16inf.u)
        {
            in.u = f16inf.u;
        }
        out = (uint16_t)(in.u >> 13U);
    }

    out |= (uint16_t)(sign >> 16U);

    return out;
}

float canardConvertFloat16ToNativeFloat(uint16_t value)
{
    CANARD_ASSERT(sizeof(float) == 4);

    union FP32
    {
        uint32_t u;
        float f;
    };

    const union FP32 magic = { (254UL - 15UL) << 23U };
    const union FP32 was_inf_nan = { (127UL + 16UL) << 23U };
    union FP32 out;

    out.u = (value & 0x7FFFU) << 13U;
    out.f *= magic.f;
    if (out.f >= was_inf_nan.f)
    {
        out.u |= 255UL << 23U;
    }
    out.u |= (value & 0x8000UL) << 16U;

    return out.f;
}
