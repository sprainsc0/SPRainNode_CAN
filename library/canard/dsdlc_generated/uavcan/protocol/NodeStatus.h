/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\3.NodeStatus.uavcan
 */

#ifndef __UAVCAN_PROTOCOL_NODESTATUS
#define __UAVCAN_PROTOCOL_NODESTATUS

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
#
# Abstract node status information.
#
# All UAVCAN nodes are required to publish this message periodically.
#

#
# Publication period may vary within these limits.
# It is NOT recommended to change it at run time.
#
uint16 MAX_BROADCASTING_PERIOD_MS = 1000
uint16 MIN_BROADCASTING_PERIOD_MS = 2

#
# If a node fails to publish this message in this amount of time, it should be considered offline.
#
uint16 OFFLINE_TIMEOUT_MS = 3000

#
# Uptime counter should never overflow.
# Other nodes may detect that a remote node has restarted when this value goes backwards.
#
uint32 uptime_sec

#
# Abstract node health.
#
uint2 HEALTH_OK         = 0     # The node is functioning properly.
uint2 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.
uint2 HEALTH_ERROR      = 2     # The node encountered a major failure.
uint2 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.
uint2 health

#
# Current mode.
#
# Mode OFFLINE can be actually reported by the node to explicitly inform other network
# participants that the sending node is about to shutdown. In this case other nodes will not
# have to wait OFFLINE_TIMEOUT_MS before they detect that the node is no longer available.
#
# Reserved values can be used in future revisions of the specification.
#
uint3 MODE_OPERATIONAL      = 0         # Normal operating mode.
uint3 MODE_INITIALIZATION   = 1         # Initialization is in progress; this mode is entered immediately after startup.
uint3 MODE_MAINTENANCE      = 2         # E.g. calibration, the bootloader is running, etc.
uint3 MODE_SOFTWARE_UPDATE  = 3         # New software/firmware is being loaded.
uint3 MODE_OFFLINE          = 7         # The node is no longer available.
uint3 mode

#
# Not used currently, keep zero when publishing, ignore when receiving.
#
uint3 sub_mode

#
# Optional, vendor-specific node status code, e.g. a fault code or a status bitmask.
#
uint16 vendor_specific_status_code
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.NodeStatus
saturated uint32 uptime_sec
saturated uint2 health
saturated uint3 mode
saturated uint3 sub_mode
saturated uint16 vendor_specific_status_code
******************************************************************************/

#define UAVCAN_PROTOCOL_NODESTATUS_ID                      3
#define UAVCAN_PROTOCOL_NODESTATUS_NAME                    "uavcan.protocol.NodeStatus"
#define UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE               (0xF0868D0C1A7C6F1ULL)

#define UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE                ((56 + 7)/8)

// Constants
#define UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS              1000 // 1000
#define UAVCAN_PROTOCOL_NODESTATUS_MIN_BROADCASTING_PERIOD_MS                 2 // 2
#define UAVCAN_PROTOCOL_NODESTATUS_OFFLINE_TIMEOUT_MS                      3000 // 3000
#define UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK                                  0 // 0
#define UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING                             1 // 1
#define UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR                               2 // 2
#define UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL                            3 // 3
#define UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL                           0 // 0
#define UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION                        1 // 1
#define UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE                           2 // 2
#define UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE                       3 // 3
#define UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE                               7 // 7

typedef struct
{
    // FieldTypes
    uint32_t   uptime_sec;                    // bit len 32
    uint8_t    health;                        // bit len 2
    uint8_t    mode;                          // bit len 3
    uint8_t    sub_mode;                      // bit len 3
    uint16_t   vendor_specific_status_code;   // bit len 16

} uavcan_protocol_NodeStatus;

static inline
uint32_t uavcan_protocol_NodeStatus_encode(uavcan_protocol_NodeStatus* source, void* msg_buf);

static inline
int32_t uavcan_protocol_NodeStatus_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_NodeStatus* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_NodeStatus_encode_internal(uavcan_protocol_NodeStatus* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_NodeStatus_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_NodeStatus* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\3.NodeStatus.uavcan
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
  * @brief uavcan_protocol_NodeStatus_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_NodeStatus_encode_internal(uavcan_protocol_NodeStatus* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->uptime_sec); // 4294967295
    offset += 32;

    source->health = CANARD_INTERNAL_SATURATE_UNSIGNED(source->health, 3)
    canardEncodeScalar(msg_buf, offset, 2, (void*)&source->health); // 3
    offset += 2;

    source->mode = CANARD_INTERNAL_SATURATE_UNSIGNED(source->mode, 7)
    canardEncodeScalar(msg_buf, offset, 3, (void*)&source->mode); // 7
    offset += 3;

    source->sub_mode = CANARD_INTERNAL_SATURATE_UNSIGNED(source->sub_mode, 7)
    canardEncodeScalar(msg_buf, offset, 3, (void*)&source->sub_mode); // 7
    offset += 3;

    canardEncodeScalar(msg_buf, offset, 16, (void*)&source->vendor_specific_status_code); // 65535
    offset += 16;

    return offset;
}

/**
  * @brief uavcan_protocol_NodeStatus_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_NodeStatus_encode(uavcan_protocol_NodeStatus* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_NodeStatus_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_NodeStatus_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_NodeStatus dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_NodeStatus_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_NodeStatus* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->uptime_sec);
    if (ret != 32)
    {
        goto uavcan_protocol_NodeStatus_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 2, false, (void*)&dest->health);
    if (ret != 2)
    {
        goto uavcan_protocol_NodeStatus_error_exit;
    }
    offset += 2;

    ret = canardDecodeScalar(transfer, offset, 3, false, (void*)&dest->mode);
    if (ret != 3)
    {
        goto uavcan_protocol_NodeStatus_error_exit;
    }
    offset += 3;

    ret = canardDecodeScalar(transfer, offset, 3, false, (void*)&dest->sub_mode);
    if (ret != 3)
    {
        goto uavcan_protocol_NodeStatus_error_exit;
    }
    offset += 3;

    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&dest->vendor_specific_status_code);
    if (ret != 16)
    {
        goto uavcan_protocol_NodeStatus_error_exit;
    }
    offset += 16;
    return offset;

uavcan_protocol_NodeStatus_error_exit:
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
  * @brief uavcan_protocol_NodeStatus_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_NodeStatus dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_NodeStatus_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_NodeStatus* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_NodeStatus); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_NodeStatus_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_PROTOCOL_NODESTATUS