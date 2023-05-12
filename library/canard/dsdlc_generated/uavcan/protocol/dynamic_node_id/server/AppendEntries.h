/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\dynamic_node_id\server\30.AppendEntries.uavcan
 */

#ifndef __UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES
#define __UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <uavcan\protocol\dynamic_node_id\server\Entry.h>

/******************************* Source text **********************************
#
# THIS DEFINITION IS SUBJECT TO CHANGE.
#
# This type is a part of the Raft consensus algorithm.
# Please refer to the specification for details.
#

#
# Given min election timeout and cluster size, the maximum recommended request interval can be derived as follows:
#
#   max recommended request interval = (min election timeout) / 2 requests / (cluster size - 1)
#
# The equation assumes that the Leader requests one Follower at a time, so that there's at most one pending call
# at any moment. Such behavior is optimal as it creates uniform bus load, but it is actually implementation-specific.
# Obviously, request interval can be lower than that if needed, but higher values are not recommended as they may
# cause Followers to initiate premature elections in case of intensive frame losses or delays.
#
# Real timeout is randomized in the range (MIN, MAX], according to the Raft paper.
#
uint16 DEFAULT_MIN_ELECTION_TIMEOUT_MS = 2000
uint16 DEFAULT_MAX_ELECTION_TIMEOUT_MS = 4000

#
# Refer to the Raft paper for explanation.
#
uint32 term
uint32 prev_log_term
uint8 prev_log_index
uint8 leader_commit

#
# Worst-case replication time per Follower can be computed as:
#
#   worst replication time = (127 log entries) * (2 trips of next_index) * (request interval per Follower)
#
Entry[<=1] entries

---

#
# Refer to the Raft paper for explanation.
#
uint32 term
bool success
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.dynamic_node_id.server.AppendEntries
saturated uint32 term
saturated uint32 prev_log_term
saturated uint8 prev_log_index
saturated uint8 leader_commit
uavcan.protocol.dynamic_node_id.server.Entry[<=1] entries
---
saturated uint32 term
saturated bool success
******************************************************************************/

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_ID 30
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_NAME "uavcan.protocol.dynamic_node_id.server.AppendEntries"
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_SIGNATURE (0x8032C7097B48A3CCULL)

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_REQUEST_MAX_SIZE ((249 + 7)/8)

// Constants
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_REQUEST_DEFAULT_MIN_ELECTION_TIMEOUT_MS       2000 // 2000
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_REQUEST_DEFAULT_MAX_ELECTION_TIMEOUT_MS       4000 // 4000

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_REQUEST_ENTRIES_MAX_LENGTH  1

typedef struct
{
    // FieldTypes
    uint32_t   term;                          // bit len 32
    uint32_t   prev_log_term;                 // bit len 32
    uint8_t    prev_log_index;                // bit len 8
    uint8_t    leader_commit;                 // bit len 8
    struct
    {
        uint8_t    len;                       // Dynamic array length
        uavcan_protocol_dynamic_node_id_server_Entry* data;                      // Dynamic Array 168bit[1] max items
    } entries;

} uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest;

static inline
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode(uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* source, void* msg_buf);

static inline
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode_internal(uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* dest, uint8_t** dyn_arr_buf, int32_t offset);

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_RESPONSE_MAX_SIZE ((33 + 7)/8)

// Constants

typedef struct
{
    // FieldTypes
    uint32_t   term;                          // bit len 32
    bool       success;                       // bit len 1

} uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse;

static inline
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode(uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* source, void* msg_buf);

static inline
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode_internal(uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\protocol\dynamic_node_id\server\30.AppendEntries.uavcan
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
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode_internal(uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->term); // 4294967295
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->prev_log_term); // 4294967295
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->prev_log_index); // 255
    offset += 8;

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->leader_commit); // 255
    offset += 8;

    // Dynamic Array (entries)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 1, (void*)&source->entries.len);
        offset += 1;
    }

    // - Add array items
    for (c = 0; c < source->entries.len; c++)
    {
        offset += uavcan_protocol_dynamic_node_id_server_Entry_encode_internal((void*)&source->entries.data[c], msg_buf, offset, 0);
    }

    return offset;
}

/**
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode(uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->term);
    if (ret != 32)
    {
        goto uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->prev_log_term);
    if (ret != 32)
    {
        goto uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 8, false, (void*)&dest->prev_log_index);
    if (ret != 8)
    {
        goto uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_error_exit;
    }
    offset += 8;

    ret = canardDecodeScalar(transfer, offset, 8, false, (void*)&dest->leader_commit);
    if (ret != 8)
    {
        goto uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_error_exit;
    }
    offset += 8;

    // Dynamic Array (entries)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len)
    {
        //  - Calculate Array length from MSG length
        dest->entries.len = ((payload_len * 8) - offset ) / 168; // 168 bit array item size
    }
    else
    {
        // - Array length 1 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 1,
                                 false,
                                 (void*)&dest->entries.len); // 0
        if (ret != 1)
        {
            goto uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_error_exit;
        }
        offset += 1;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->entries.data = (uavcan_protocol_dynamic_node_id_server_Entry*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->entries.len; c++)
    {
        offset += uavcan_protocol_dynamic_node_id_server_Entry_decode_internal(transfer,
                                                0,
                                                (void*)&dest->entries.data[c],
                                                dyn_arr_buf,
                                                offset);
    }
    return offset;

uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_error_exit:
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
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_dynamic_node_id_server_AppendEntriesRequest_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

/**
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode_internal(uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->term); // 4294967295
    offset += 32;

    source->success = CANARD_INTERNAL_SATURATE_UNSIGNED(source->success, 0)
    canardEncodeScalar(msg_buf, offset, 1, (void*)&source->success); // 0
    offset += 1;

    return offset;
}

/**
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode(uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->term);
    if (ret != 32)
    {
        goto uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 1, false, (void*)&dest->success);
    if (ret != 1)
    {
        goto uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_error_exit;
    }
    offset += 1;
    return offset;

uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_error_exit:
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
  * @brief uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_protocol_dynamic_node_id_server_AppendEntriesResponse_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES