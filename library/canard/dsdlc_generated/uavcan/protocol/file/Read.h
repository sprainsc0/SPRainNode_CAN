/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\file\48.Read.uavcan
 */

#ifndef __UAVCAN_PROTOCOL_FILE_READ
#define __UAVCAN_PROTOCOL_FILE_READ

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <uavcan\protocol\file\Error.h>
#include <uavcan\protocol\file\Path.h>

/******************************* Source text **********************************
#
# Read file from a remote node.
#
# There are two possible outcomes of a successful service call:
#  1. Data array size equals its capacity. This means that the end of the file is not reached yet.
#  2. Data array size is less than its capacity, possibly zero. This means that the end of file is reached.
#
# Thus, if the client needs to fetch the entire file, it should repeatedly call this service while increasing the
# offset, until incomplete data is returned.
#
# If the object pointed by 'path' cannot be read (e.g. it is a directory or it does not exist), appropriate error code
# will be returned, and data array will be empty.
#

uint40 offset

Path path

---

Error error

uint8[<=256] data
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.file.Read
saturated uint40 offset
uavcan.protocol.file.Path path
---
uavcan.protocol.file.Error error
saturated uint8[<=256] data
******************************************************************************/

#define UAVCAN_PROTOCOL_FILE_READ_ID                       48
#define UAVCAN_PROTOCOL_FILE_READ_NAME                     "uavcan.protocol.file.Read"
#define UAVCAN_PROTOCOL_FILE_READ_SIGNATURE                (0x8DCDCA939F33F678ULL)

#define UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE         ((1648 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    uint64_t   offset;                        // bit len 40
    uavcan_protocol_file_Path path;                          //

} uavcan_protocol_file_ReadRequest;

static inline
uint32_t uavcan_protocol_file_ReadRequest_encode(uavcan_protocol_file_ReadRequest* source, void* msg_buf);

static inline
int32_t uavcan_protocol_file_ReadRequest_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_file_ReadRequest* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_file_ReadRequest_encode_internal(uavcan_protocol_file_ReadRequest* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_file_ReadRequest_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_file_ReadRequest* dest, uint8_t** dyn_arr_buf, int32_t offset);

#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_MAX_SIZE        ((2073 + 7)/8)

// Constants

#define UAVCAN_PROTOCOL_FILE_READ_RESPONSE_DATA_MAX_LENGTH                               256

typedef struct
{
    // FieldTypes
    uavcan_protocol_file_Error error;                         //
    struct
    {
        uint16_t    len;                       // Dynamic array length
        uint8_t*   data;                      // Dynamic Array 8bit[256] max items
    } data;

} uavcan_protocol_file_ReadResponse;

static inline
uint32_t uavcan_protocol_file_ReadResponse_encode(uavcan_protocol_file_ReadResponse* source, void* msg_buf);

static inline
int32_t uavcan_protocol_file_ReadResponse_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_file_ReadResponse* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_file_ReadResponse_encode_internal(uavcan_protocol_file_ReadResponse* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_file_ReadResponse_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_file_ReadResponse* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\file\48.Read.uavcan
 */

#ifndef CANARD_INTERNAL_SATURATE
#define CANARD_INTERNAL_SATURATE(x, max) ( ((x) > max) ? max : ( (-(x) > max) ? (-max) : (x) ) );
#endif

#ifndef CANARD_INTERNAL_SATURATE_UNSIGNED
#define CANARD_INTERNAL_SATURATE_UNSIGNED(x, max) ( ((x) > max) ? max : (x) );
#endif

#if defined(__GNUC__)
# define CANARD_MAYBE_UNUSED(x) x __attribute__((unused))
#else
# define CANARD_MAYBE_UNUSED(x) x
#endif

/**
  * @brief uavcan_protocol_file_ReadRequest_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_file_ReadRequest_encode_internal(uavcan_protocol_file_ReadRequest* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    source->offset = CANARD_INTERNAL_SATURATE_UNSIGNED(source->offset, 1099511627775)
    canardEncodeScalar(msg_buf, offset, 40, (void*)&source->offset); // 1099511627775
    offset += 40;

    // Compound
    offset = uavcan_protocol_file_Path_encode_internal(&source->path, msg_buf, offset, 0);

    return offset;
}

/**
  * @brief uavcan_protocol_file_ReadRequest_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_file_ReadRequest_encode(uavcan_protocol_file_ReadRequest* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_file_ReadRequest_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_file_ReadRequest_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_file_ReadRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_file_ReadRequest_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_file_ReadRequest* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, offset, 40, false, (void*)&dest->offset);
    if (ret != 40)
    {
        goto uavcan_protocol_file_ReadRequest_error_exit;
    }
    offset += 40;

    // Compound
    offset = uavcan_protocol_file_Path_decode_internal(transfer, 0, &dest->path, dyn_arr_buf, offset);
    if (offset < 0)
    {
        ret = offset;
        goto uavcan_protocol_file_ReadRequest_error_exit;
    }
    return offset;

uavcan_protocol_file_ReadRequest_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}

/**
  * @brief uavcan_protocol_file_ReadRequest_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_file_ReadRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_file_ReadRequest_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_file_ReadRequest* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_file_ReadRequest); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_file_ReadRequest_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

/**
  * @brief uavcan_protocol_file_ReadResponse_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_file_ReadResponse_encode_internal(uavcan_protocol_file_ReadResponse* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    // Compound
    offset = uavcan_protocol_file_Error_encode_internal(&source->error, msg_buf, offset, 0);

    // Dynamic Array (data)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 9, (void*)&source->data.len);
        offset += 9;
    }

    // - Add array items
    for (c = 0; c < source->data.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           8,
                           (void*)(source->data.data + c));// 255
        offset += 8;
    }

    return offset;
}

/**
  * @brief uavcan_protocol_file_ReadResponse_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_file_ReadResponse_encode(uavcan_protocol_file_ReadResponse* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_file_ReadResponse_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_file_ReadResponse_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_file_ReadResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_file_ReadResponse_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_file_ReadResponse* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;

    // Compound
    offset = uavcan_protocol_file_Error_decode_internal(transfer, 0, &dest->error, dyn_arr_buf, offset);
    if (offset < 0)
    {
        ret = offset;
        goto uavcan_protocol_file_ReadResponse_error_exit;
    }

    // Dynamic Array (data)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len)
    {
        //  - Calculate Array length from MSG length
        dest->data.len = ((payload_len * 8) - offset ) / 8; // 8 bit array item size
    }
    else
    {
        // - Array length 9 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 9,
                                 false,
                                 (void*)&dest->data.len); // 255
        if (ret != 9)
        {
            goto uavcan_protocol_file_ReadResponse_error_exit;
        }
        offset += 9;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->data.data = (uint8_t*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->data.len; c++)
    {
        if (dyn_arr_buf)
        {
            ret = canardDecodeScalar(transfer,
                                     offset,
                                     8,
                                     false,
                                     (void*)*dyn_arr_buf); // 255
            if (ret != 8)
            {
                goto uavcan_protocol_file_ReadResponse_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((uint8_t*)*dyn_arr_buf) + 1);
        }
        offset += 8;
    }
    return offset;

uavcan_protocol_file_ReadResponse_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}

/**
  * @brief uavcan_protocol_file_ReadResponse_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_file_ReadResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_file_ReadResponse_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_file_ReadResponse* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_file_ReadResponse); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_file_ReadResponse_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_PROTOCOL_FILE_READ