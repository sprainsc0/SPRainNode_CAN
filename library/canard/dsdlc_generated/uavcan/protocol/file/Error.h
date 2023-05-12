/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\file\Error.uavcan
 */

#ifndef __UAVCAN_PROTOCOL_FILE_ERROR
#define __UAVCAN_PROTOCOL_FILE_ERROR

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Nested type.
# File operation result code.
#

int16 OK                = 0
int16 UNKNOWN_ERROR     = 32767

int16 NOT_FOUND         = 2
int16 IO_ERROR          = 5
int16 ACCESS_DENIED     = 13
int16 IS_DIRECTORY      = 21 # I.e. attempt to read/write on a path that points to a directory
int16 INVALID_VALUE     = 22 # E.g. file name is not valid for the target file system
int16 FILE_TOO_LARGE    = 27
int16 OUT_OF_SPACE      = 28
int16 NOT_IMPLEMENTED   = 38

int16 value
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.file.Error
saturated int16 value
******************************************************************************/

#define UAVCAN_PROTOCOL_FILE_ERROR_NAME                    "uavcan.protocol.file.Error"
#define UAVCAN_PROTOCOL_FILE_ERROR_SIGNATURE               (0xA83071FFEA4FAE15ULL)

#define UAVCAN_PROTOCOL_FILE_ERROR_MAX_SIZE                ((16 + 7)/8)

// Constants
#define UAVCAN_PROTOCOL_FILE_ERROR_OK                                         0 // 0
#define UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR                          32767 // 32767
#define UAVCAN_PROTOCOL_FILE_ERROR_NOT_FOUND                                  2 // 2
#define UAVCAN_PROTOCOL_FILE_ERROR_IO_ERROR                                   5 // 5
#define UAVCAN_PROTOCOL_FILE_ERROR_ACCESS_DENIED                             13 // 13
#define UAVCAN_PROTOCOL_FILE_ERROR_IS_DIRECTORY                              21 // 21
#define UAVCAN_PROTOCOL_FILE_ERROR_INVALID_VALUE                             22 // 22
#define UAVCAN_PROTOCOL_FILE_ERROR_FILE_TOO_LARGE                            27 // 27
#define UAVCAN_PROTOCOL_FILE_ERROR_OUT_OF_SPACE                              28 // 28
#define UAVCAN_PROTOCOL_FILE_ERROR_NOT_IMPLEMENTED                           38 // 38

typedef struct
{
    // FieldTypes
    int16_t    value;                         // bit len 16

} uavcan_protocol_file_Error;

static inline
uint32_t uavcan_protocol_file_Error_encode(uavcan_protocol_file_Error* source, void* msg_buf);

static inline
int32_t uavcan_protocol_file_Error_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_file_Error* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_file_Error_encode_internal(uavcan_protocol_file_Error* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_file_Error_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_file_Error* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\file\Error.uavcan
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
  * @brief uavcan_protocol_file_Error_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_file_Error_encode_internal(uavcan_protocol_file_Error* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    canardEncodeScalar(msg_buf, offset, 16, (void*)&source->value); // 32767
    offset += 16;

    return offset;
}

/**
  * @brief uavcan_protocol_file_Error_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_file_Error_encode(uavcan_protocol_file_Error* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_file_Error_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_file_Error_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_file_Error dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_file_Error_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_file_Error* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, offset, 16, true, (void*)&dest->value);
    if (ret != 16)
    {
        goto uavcan_protocol_file_Error_error_exit;
    }
    offset += 16;
    return offset;

uavcan_protocol_file_Error_error_exit:
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
  * @brief uavcan_protocol_file_Error_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_file_Error dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_file_Error_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_file_Error* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_file_Error); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_file_Error_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_PROTOCOL_FILE_ERROR