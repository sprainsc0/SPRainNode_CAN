#ifndef CANLINK_PARAM_H_
#define CANLINK_PARAM_H_

#include <stdint.h>
#include "cmsis_os.h"
#include <ipccore.h>
#include <ipcpush.h>
#include <ipcpull.h>
#include "param.h"
#include "uPerf.h"
#include "can.h"

#include <topics/parameter_update.h>
#include <topics/uavcan_parameter_request.h>
#include <topics/uavcan_parameter_value.h>

#include <uavcan/protocol/param/GetSet.h>

#define HASH_PARAM "_HASH_CHECK"

class CanLink;

class CanParam
{
public:
	CanParam(CanLink *can);

	void uavcan_param(void);
	void send_param(param_t param);
	void send_hash(void);
    bool send_one();

protected:
	CanLink *_can;

private:
	int send_all_index;
	uint8_t transfer_id;

	IPCPull _param_req;
};


#endif /* MAVLINK_STREAM_H_ */
