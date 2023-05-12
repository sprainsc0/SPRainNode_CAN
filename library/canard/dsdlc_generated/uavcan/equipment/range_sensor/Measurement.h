/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\equipment\range_sensor\1050.Measurement.uavcan
 */

#ifndef __UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT
#define __UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <uavcan\Timestamp.h>

/******************************* Source text **********************************
# 1050
# Generic narrow-beam range sensor data.
#

uavcan.Timestamp timestamp

uint5 SENSOR_TYPE_UNDEFINED = 0
uint5 SENSOR_TYPE_SONAR     = 1
uint5 SENSOR_TYPE_LIDAR     = 2
uint5 SENSOR_TYPE_RADAR     = 3
uint5 sensor_type

uint3 READING_TYPE_UNDEFINED   = 0   # Range is unknown
uint3 READING_TYPE_VALID_RANGE = 1   # Range field contains valid distance
uint3 READING_TYPE_TOO_CLOSE   = 2   # Range field contains min range for the sensor
uint3 READING_TYPE_TOO_FAR     = 3   # Range field contains max range for the sensor
uint3 reading_type

float16  rng_max
float16  rng_min
float32 range                        # Meters
uint16  qualty
float16 covariance

float16[3] pos_offset
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.range_sensor.Measurement
uavcan.Timestamp timestamp
saturated uint5 sensor_type
saturated uint3 reading_type
saturated float16 rng_max
saturated float16 rng_min
saturated float32 range
saturated uint16 qualty
saturated float16 covariance
saturated float16[3] pos_offset
******************************************************************************/

#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID       1050
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_NAME     "uavcan.equipment.range_sensor.Measurement"
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE (0xBD87A7A603C67BD1ULL)

#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE ((208 + 7)/8)

// Constants
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_UNDEFINED          0 // 0
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR           1 // 1
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR           2 // 2
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR           3 // 3
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED          0 // 0
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE          1 // 1
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE          2 // 2
#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR          3 // 3

#define UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_POS_OFFSET_LENGTH                      3

typedef struct
{
    // FieldTypes
    uavcan_Timestamp timestamp;                     //
    uint8_t    sensor_type;                   // bit len 5
    uint8_t    reading_type;                  // bit len 3
    float      rng_max;                       // float16 Saturate
    float      rng_min;                       // float16 Saturate
    float      range;                         // float32 Saturate
    uint16_t   qualty;                        // bit len 16
    float      covariance;                    // float16 Saturate
    float      pos_offset[3];                 // Static Array 16bit[3] max items

} uavcan_equipment_range_sensor_Measurement;

static inline
uint32_t uavcan_equipment_range_sensor_Measurement_encode(uavcan_equipment_range_sensor_Measurement* source, void* msg_buf);

static inline
int32_t uavcan_equipment_range_sensor_Measurement_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_equipment_range_sensor_Measurement* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_equipment_range_sensor_Measurement_encode_internal(uavcan_equipment_range_sensor_Measurement* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_equipment_range_sensor_Measurement_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_equipment_range_sensor_Measurement* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: f:\SPRainCore\aircraft_h7\module\canard\dsdl\uavcan\equipment\range_sensor\1050.Measurement.uavcan
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
  * @brief uavcan_equipment_range_sensor_Measurement_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_equipment_range_sensor_Measurement_encode_internal(uavcan_equipment_range_sensor_Measurement* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    // Compound
    offset = uavcan_Timestamp_encode_internal(&source->timestamp, msg_buf, offset, 0);
    source->sensor_type = CANARD_INTERNAL_SATURATE_UNSIGNED(source->sensor_type, 31)
    canardEncodeScalar(msg_buf, offset, 5, (void*)&source->sensor_type); // 31
    offset += 5;

    source->reading_type = CANARD_INTERNAL_SATURATE_UNSIGNED(source->reading_type, 7)
    canardEncodeScalar(msg_buf, offset, 3, (void*)&source->reading_type); // 7
    offset += 3;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->rng_max);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->rng_max;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->rng_min);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->rng_min;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;
    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->range); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 16, (void*)&source->qualty); // 65535
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->covariance);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->covariance;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;
    // Static array (pos_offset)
    for (c = 0; c < 3; c++)
    {
#ifndef CANARD_USE_FLOAT16_CAST
        uint16_t tmpe_float = canardConvertNativeFloatToFloat16(source->pos_offset[c]);
#else
        CANARD_USE_FLOAT16_CAST tmpe_float = (CANARD_USE_FLOAT16_CAST)source->pos_offset;
#endif
        canardEncodeScalar(msg_buf, offset, 16, (void*)&tmpe_float); // 32767
        offset += 16;
    }

    return offset;
}

/**
  * @brief uavcan_equipment_range_sensor_Measurement_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_equipment_range_sensor_Measurement_encode(uavcan_equipment_range_sensor_Measurement* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_equipment_range_sensor_Measurement_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_equipment_range_sensor_Measurement_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_range_sensor_Measurement dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_range_sensor_Measurement_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_range_sensor_Measurement* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    // Compound
    offset = uavcan_Timestamp_decode_internal(transfer, 0, &dest->timestamp, dyn_arr_buf, offset);
    if (offset < 0)
    {
        ret = offset;
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }

    ret = canardDecodeScalar(transfer, offset, 5, false, (void*)&dest->sensor_type);
    if (ret != 5)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
    offset += 5;

    ret = canardDecodeScalar(transfer, offset, 3, false, (void*)&dest->reading_type);
    if (ret != 3)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
    offset += 3;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->rng_max = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->rng_max = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->rng_min = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->rng_min = (float)tmp_float;
#endif
    offset += 16;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->range);
    if (ret != 32)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&dest->qualty);
    if (ret != 16)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_range_sensor_Measurement_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->covariance = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->covariance = (float)tmp_float;
#endif
    offset += 16;

    // Static array (pos_offset)
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
            goto uavcan_equipment_range_sensor_Measurement_error_exit;
        }
#ifndef CANARD_USE_FLOAT16_CAST
        dest->pos_offset[c] = canardConvertFloat16ToNativeFloat(tmpe_float);
#else
        dest->pos_offset[c] = (float)tmpe_float;
#endif
        offset += 16;
    }
    return offset;

uavcan_equipment_range_sensor_Measurement_error_exit:
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
  * @brief uavcan_equipment_range_sensor_Measurement_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_range_sensor_Measurement dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_range_sensor_Measurement_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_equipment_range_sensor_Measurement* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_equipment_range_sensor_Measurement); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_equipment_range_sensor_Measurement_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT