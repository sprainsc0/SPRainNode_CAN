#ifndef CANLINK_MESSAGES_H_
#define CANLINK_MESSAGES_H_

#include "canlink_stream.h"

class CANStreamListItem
{

public:
	CanStream *(*new_instance)(CanLink *can);
	const char *(*get_name)();
	uint32_t (*get_id)();

	CANStreamListItem(CanStream * (*inst)(CanLink *can), const char *(*name)(), uint32_t (*id)()) :
		new_instance(inst),
		get_name(name),
		get_id(id) {}

};

const char *get_message_name(const uint32_t link_id);
CanStream *create_message_stream(const char *link_name, CanLink *can);

#endif /* MAVLINK_MESSAGES_H_ */
