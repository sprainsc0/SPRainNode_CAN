/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\enumeration\15.Begin.uavcan
 */

#ifndef __UAVCAN_PROTOCOL_ENUMERATION_BEGIN
#define __UAVCAN_PROTOCOL_ENUMERATION_BEGIN

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# This service instructs the node to begin the process of automated enumeration.
#

#
# The node will automatically leave enumeration mode upon expiration of this timeout.
#
uint16 TIMEOUT_CANCEL   = 0     # Stop enumeration immediately
uint16 TIMEOUT_INFINITE = 65535 # Do not stop until explicitly requested
uint16 timeout_sec              # [Seconds]

#
# Name of the parameter to enumerate, e.g. ESC index.
# If the name is left empty, the node will infer the parameter name automatically (autodetect).
# It is highly recommended to always use autodetection in order to avoid dependency on hard-coded parameter names,
# and also allow the enumeratee to possibly enumerate multiple different parameters at once.
# The rule of thumb is to always leave this parameter empty unless you really know what you're doing.
#
uint8[<=92] parameter_name

---

uint8 ERROR_OK                  = 0     # Success
uint8 ERROR_INVALID_MODE        = 1     # The node cannot perform enumeration in its current operating mode
uint8 ERROR_INVALID_PARAMETER   = 2     # The node cannot enumerate on the requested parameter, or it doesn't exist
uint8 ERROR_UNSUPPORTED         = 3     # The node cannot perform enumeration in its current configuration
uint8 ERROR_UNKNOWN             = 255   # Generic error
uint8 error
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.enumeration.Begin
saturated uint16 timeout_sec
saturated uint8[<=92] parameter_name
---
saturated uint8 error
******************************************************************************/

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_ID               15
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_NAME             "uavcan.protocol.enumeration.Begin"
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_SIGNATURE        (0x196AE06426A3B5D8ULL)

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_REQUEST_MAX_SIZE ((759 + 7)/8)

// Constants
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_REQUEST_TIMEOUT_CANCEL              0 // 0
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_REQUEST_TIMEOUT_INFINITE        65535 // 65535

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_REQUEST_PARAMETER_NAME_MAX_LENGTH              92

typedef struct
{
    // FieldTypes
    uint16_t   timeout_sec;                   // bit len 16
    struct
    {
        uint8_t    len;                       // Dynamic array length
        uint8_t*   data;                      // Dynamic Array 8bit[92] max items
    } parameter_name;

} uavcan_protocol_enumeration_BeginRequest;

static inline
uint32_t uavcan_protocol_enumeration_BeginRequest_encode(uavcan_protocol_enumeration_BeginRequest* source, void* msg_buf);

static inline
int32_t uavcan_protocol_enumeration_BeginRequest_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_enumeration_BeginRequest* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_enumeration_BeginRequest_encode_internal(uavcan_protocol_enumeration_BeginRequest* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_enumeration_BeginRequest_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_enumeration_BeginRequest* dest, uint8_t** dyn_arr_buf, int32_t offset);

#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_MAX_SIZE ((8 + 7)/8)

// Constants
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_OK                   0 // 0
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_INVALID_MODE          1 // 1
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_INVALID_PARAMETER          2 // 2
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_UNSUPPORTED          3 // 3
#define UAVCAN_PROTOCOL_ENUMERATION_BEGIN_RESPONSE_ERROR_UNKNOWN            255 // 255

typedef struct
{
    // FieldTypes
    uint8_t    error;                         // bit len 8

} uavcan_protocol_enumeration_BeginResponse;

static inline
uint32_t uavcan_protocol_enumeration_BeginResponse_encode(uavcan_protocol_enumeration_BeginResponse* source, void* msg_buf);

static inline
int32_t uavcan_protocol_enumeration_BeginResponse_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_enumeration_BeginResponse* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_enumeration_BeginResponse_encode_internal(uavcan_protocol_enumeration_BeginResponse* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_enumeration_BeginResponse_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_enumeration_BeginResponse* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\enumeration\15.Begin.uavcan
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
  * @brief uavcan_protocol_enumeration_BeginRequest_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_enumeration_BeginRequest_encode_internal(uavcan_protocol_enumeration_BeginRequest* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    canardEncodeScalar(msg_buf, offset, 16, (void*)&source->timeout_sec); // 65535
    offset += 16;

    // Dynamic Array (parameter_name)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 7, (void*)&source->parameter_name.len);
        offset += 7;
    }

    // - Add array items
    for (c = 0; c < source->parameter_name.len; c++)
    {
        canardEncodeScalar(msg_buf,
                           offset,
                           8,
                           (void*)(source->parameter_name.data + c));// 255
        offset += 8;
    }

    return offset;
}

/**
  * @brief uavcan_protocol_enumeration_BeginRequest_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_enumeration_BeginRequest_encode(uavcan_protocol_enumeration_BeginRequest* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_enumeration_BeginRequest_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_enumeration_BeginRequest_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_enumeration_BeginRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_enumeration_BeginRequest_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_enumeration_BeginRequest* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&dest->timeout_sec);
    if (ret != 16)
    {
        goto uavcan_protocol_enumeration_BeginRequest_error_exit;
    }
    offset += 16;

    // Dynamic Array (parameter_name)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len)
    {
        //  - Calculate Array length from MSG length
        dest->parameter_name.len = ((payload_len * 8) - offset ) / 8; // 8 bit array item size
    }
    else
    {
        // - Array length 7 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 7,
                                 false,
                                 (void*)&dest->parameter_name.len); // 255
        if (ret != 7)
        {
            goto uavcan_protocol_enumeration_BeginRequest_error_exit;
        }
        offset += 7;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->parameter_name.data = (uint8_t*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->parameter_name.len; c++)
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
                goto uavcan_protocol_enumeration_BeginRequest_error_exit;
            }
            *dyn_arr_buf = (uint8_t*)(((uint8_t*)*dyn_arr_buf) + 1);
        }
        offset += 8;
    }
    return offset;

uavcan_protocol_enumeration_BeginRequest_error_exit:
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
  * @brief uavcan_protocol_enumeration_BeginRequest_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_enumeration_BeginRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_enumeration_BeginRequest_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_enumeration_BeginRequest* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_enumeration_BeginRequest); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_enumeration_BeginRequest_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

/**
  * @brief uavcan_protocol_enumeration_BeginResponse_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_enumeration_BeginResponse_encode_internal(uavcan_protocol_enumeration_BeginResponse* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->error); // 255
    offset += 8;

    return offset;
}

/**
  * @brief uavcan_protocol_enumeration_BeginResponse_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_enumeration_BeginResponse_encode(uavcan_protocol_enumeration_BeginResponse* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_enumeration_BeginResponse_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_enumeration_BeginResponse_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_enumeration_BeginResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_enumeration_BeginResponse_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_enumeration_BeginResponse* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, offset, 8, false, (void*)&dest->error);
    if (ret != 8)
    {
        goto uavcan_protocol_enumeration_BeginResponse_error_exit;
    }
    offset += 8;
    return offset;

uavcan_protocol_enumeration_BeginResponse_error_exit:
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
  * @brief uavcan_protocol_enumeration_BeginResponse_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_enumeration_BeginResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_enumeration_BeginResponse_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_enumeration_BeginResponse* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_enumeration_BeginResponse); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_enumeration_BeginResponse_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_PROTOCOL_ENUMERATION_BEGIN