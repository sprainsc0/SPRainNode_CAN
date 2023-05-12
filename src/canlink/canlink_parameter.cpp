#include <stdlib.h>

#include "canlink_parameter.h"
#include "canlink.h"
#include "debug.h"

CanParam::CanParam(CanLink *can) :
	_can(can),
	send_all_index(-1),
	transfer_id(0),
	_param_req(IPC_ID(uavcan_parameter_request))
{
}


void CanParam::uavcan_param(void)
{
	uavcan_parameter_request_s req;
    if(_param_req.update_if_changed(&req)) {
		// Request all parameter
        if(req.message_type == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_PARAM_REQUEST_LIST) {
            if (send_all_index < 0) {
                send_all_index = PARAM_HASH;
            } else {
                send_all_index = 0;
            }
        }
		// Set parameter
        else if(req.message_type == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_PARAM_SET && req.param_type != PROTOCOL_PARAM_VALUE_EMPTY) {
			char name[17];
			strncpy(name, req.param_id, 16);
			name[16] = '\0';

			if (strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {
				send_all_index = -1;
				/* No other action taken, return */
				return;
			}

			param_t param = param_find_no_notification(name);

			if (param != PARAM_INVALID) {
				if(req.param_type == PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
					int curr_val;
					param_get(param, &curr_val);
					param_set(param, &(req.int_value));
				} else if(req.param_type == PROTOCOL_PARAM_VALUE_REAL_VALUE) {
					float curr_val;
					param_get(param, &curr_val);
					param_set(param, &(req.real_value));
				}
			}
        }

		// Read parameter
		else if(req.message_type == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_PARAM_READ) {
			if (strncmp(req.param_id, HASH_PARAM, 16) == 0) {
				send_hash();
			} else {
				char name[17];
				strncpy(name, req.param_id, 16);
				name[16] = '\0';

				send_param(param_find_no_notification(name));
			}
		}
    }
}

bool CanParam::send_one()
{
	if (send_all_index >= 0) {
		/* send all parameters if requested, but only after the system has booted */

		/* The first thing we send is a hash of all values for the ground
		 * station to try and quickly load a cached copy of our params
		 */
		if (send_all_index == PARAM_HASH) {
			send_hash();

			/* after this we should start sending all params */
			send_all_index = 0;

			/* No further action, return now */
			return true;
		}

		/* look for the first parameter which is used */
		param_t p;

		do {
			/* walk through all parameters, including unused ones */
			p = param_for_index(send_all_index);
			send_all_index++;
		} while (p != PARAM_INVALID && !param_used(p));

		if (p != PARAM_INVALID) {
			send_param(p);
		}

		if ((p == PARAM_INVALID) || (send_all_index >= (int) param_count())) {
			send_all_index = -1;
			return false;

		} else {
			return true;
		}

	} else if (send_all_index == PARAM_HASH && micros() > 20 * 1000 * 1000) {
		/* the boot did not seem to ever complete, warn user and set boot complete */
		// _mavlink->send_statustext_critical("WARNING: SYSTEM BOOT INCOMPLETE. CHECK CONFIG.");
	}

	return false;
}

void CanParam::send_param(param_t param)
{
	if (param == PARAM_INVALID) {
		return;
	}

	uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE];
        
	memset(buffer, 0, UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE);

	const char *pa_name = param_name(param);
	param_type_t type = param_type(param);
	uint8_t name_buf[20];

	memset(name_buf, 0, sizeof(name_buf));

	uavcan_protocol_param_GetSetRequest param_res;
	param_res.index = param_get_used_index(param);
	param_res.count = param_count_used();
	param_res.name.data = name_buf;
	param_res.name.len  = strlen(pa_name)+1;
	memcpy(param_res.name.data, pa_name, strlen(pa_name));
    param_res.name.data[param_res.name.len] = '\0';

	if(type == PARAM_TYPE_INT32) {
		param_res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
		param_get(param, &param_res.value.integer_value);
	} else if(type == PARAM_TYPE_FLOAT) {
		param_res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
		param_get(param, &param_res.value.real_value);
	} else {
		return;
	}
	
	const uint32_t total_size = uavcan_protocol_param_GetSetRequest_encode(&param_res, buffer);

	const CanardTransfer transfer = {
		/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
		/*priority       */ CanardPrioritySlow,
		/*transfer_kind  */ CanardTransferKindResponse,
		/*port_id        */ UAVCAN_PROTOCOL_PARAM_GETSET_ID, // This is the subject-ID.
		/*remote_node_id */ CANLINK_ID_FCU,                  // Messages cannot be unicast, so use UNSET.
		/*transfer_id    */ transfer_id,
		/*payload_size   */ total_size,
		/*payload        */ buffer,
	};

	++transfer_id;

	const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

	// Info_Debug("PARAM2 %d", resp_res);

	if (resp_res <= 0) {
		return;
	}
}

void CanParam::send_hash(void)
{
	uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE];
        
	memset(buffer, 0, UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE);

	uint32_t hash = param_hash_check();
	uint8_t name_buf[20];

	memset(name_buf, 0, sizeof(name_buf));

	uavcan_protocol_param_GetSetRequest param_res;
	param_res.index = -1;
	param_res.count = param_count_used();
	param_res.name.data = name_buf;
	param_res.name.len  = strlen(HASH_PARAM)+1;
	memcpy(param_res.name.data, HASH_PARAM, strlen(HASH_PARAM));

	param_res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
	param_res.value.integer_value = hash;

	
	const uint32_t total_size = uavcan_protocol_param_GetSetRequest_encode(&param_res, buffer);

	const CanardTransfer transfer = {
		/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
		/*priority       */ CanardPrioritySlow,
		/*transfer_kind  */ CanardTransferKindResponse,
		/*port_id        */ UAVCAN_PROTOCOL_PARAM_GETSET_ID, // This is the subject-ID.
		/*remote_node_id */ CANLINK_ID_FCU,                  // Messages cannot be unicast, so use UNSET.
		/*transfer_id    */ transfer_id,
		/*payload_size   */ total_size,
		/*payload        */ buffer,
	};

	++transfer_id;

	const int32_t resp_res = canardTxPush(_can->canIns(), &transfer);

	if (resp_res <= 0) {
		return;
	}
}