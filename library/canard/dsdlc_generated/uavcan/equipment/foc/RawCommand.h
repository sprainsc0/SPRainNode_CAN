/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\equipment\foc\1120.RawCommand.uavcan
 */

#ifndef __UAVCAN_EQUIPMENT_FOC_RAWCOMMAND
#define __UAVCAN_EQUIPMENT_FOC_RAWCOMMAND

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Raw FOC command normalized into [-8192, 8191]; negative values indicate reverse rotation.
# The FOC should normalize the setpoint into its effective input range.
# Non-zero setpoint value below minimum should be interpreted as min valid setpoint for the given motor.
#

# FOC run mode
uint3 FOC_MODE_IDLE        = 0
uint3 FOC_MODE_OPENLOOP    = 1
uint3 FOC_MODE_TORQUE      = 2
uint3 FOC_MODE_SPEED       = 3
uint3 FOC_MODE_POSITION    = 4
uint3 mode

float16[3] target
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.foc.RawCommand
saturated uint3 mode
saturated float16[3] target
******************************************************************************/

#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_ID                 1120
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_NAME               "uavcan.equipment.foc.RawCommand"
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_SIGNATURE          (0x829162135B6CF85FULL)

#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_MAX_SIZE           ((51 + 7)/8)

// Constants
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_FOC_MODE_IDLE                         0 // 0
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_FOC_MODE_OPENLOOP                     1 // 1
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_FOC_MODE_TORQUE                       2 // 2
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_FOC_MODE_SPEED                        3 // 3
#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_FOC_MODE_POSITION                     4 // 4

#define UAVCAN_EQUIPMENT_FOC_RAWCOMMAND_TARGET_LENGTH                                    3

typedef struct
{
    // FieldTypes
    uint8_t    mode;                          // bit len 3
    float      target[3];                     // Static Array 16bit[3] max items

} uavcan_equipment_foc_RawCommand;

static inline
uint32_t uavcan_equipment_foc_RawCommand_encode(uavcan_equipment_foc_RawCommand* source, void* msg_buf);

static inline
int32_t uavcan_equipment_foc_RawCommand_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_equipment_foc_RawCommand* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_equipment_foc_RawCommand_encode_internal(uavcan_equipment_foc_RawCommand* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_equipment_foc_RawCommand_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_equipment_foc_RawCommand* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\equipment\foc\1120.RawCommand.uavcan
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
  * @brief uavcan_equipment_foc_RawCommand_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_equipment_foc_RawCommand_encode_internal(uavcan_equipment_foc_RawCommand* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    source->mode = CANARD_INTERNAL_SATURATE_UNSIGNED(source->mode, 7)
    canardEncodeScalar(msg_buf, offset, 3, (void*)&source->mode); // 7
    offset += 3;

    // Static array (target)
    for (c = 0; c < 3; c++)
    {
#ifndef CANARD_USE_FLOAT16_CAST
        uint16_t tmpe_float = canardConvertNativeFloatToFloat16(source->target[c]);
#else
        CANARD_USE_FLOAT16_CAST tmpe_float = (CANARD_USE_FLOAT16_CAST)source->target;
#endif
        canardEncodeScalar(msg_buf, offset, 16, (void*)&tmpe_float); // 32767
        offset += 16;
    }

    return offset;
}

/**
  * @brief uavcan_equipment_foc_RawCommand_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_equipment_foc_RawCommand_encode(uavcan_equipment_foc_RawCommand* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_equipment_foc_RawCommand_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_equipment_foc_RawCommand_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_foc_RawCommand dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_foc_RawCommand_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_foc_RawCommand* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 3, false, (void*)&dest->mode);
    if (ret != 3)
    {
        goto uavcan_equipment_foc_RawCommand_error_exit;
    }
    offset += 3;

    // Static array (target)
    for (c = 0; c < 3; c++)
    {
#ifndef CANARD_USE_FLOAT16_CAST
        uint16_t tmpe_float = 0;
#else
        CANARD_USE_FLOAT16_CAST tmpe_float = 0;
#endif
        ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmpe_float);
        if (ret != 16)
        {
            goto uavcan_equipment_foc_RawCommand_error_exit;
        }
#ifndef CANARD_USE_FLOAT16_CAST
        dest->target[c] = canardConvertFloat16ToNativeFloat(tmpe_float);
#else
        dest->target[c] = (float)tmpe_float;
#endif
        offset += 16;
    }
    return offset;

uavcan_equipment_foc_RawCommand_error_exit:
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
  * @brief uavcan_equipment_foc_RawCommand_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_foc_RawCommand dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_foc_RawCommand_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_equipment_foc_RawCommand* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_equipment_foc_RawCommand); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_equipment_foc_RawCommand_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_EQUIPMENT_FOC_RAWCOMMAND