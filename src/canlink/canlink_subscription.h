#ifndef CANLINK_SUBSCRIPTION_H_
#define CANLINK_SUBSCRIPTION_H_

#include "canlink_stream.h"

class CANStreamSubscriptionItem
{

public:
	CanStream *(*new_instance)(CanLink *can);
	const char *(*get_name)();
	uint32_t (*get_id)();

	CANStreamSubscriptionItem(CanStream * (*inst)(CanLink *can), const char *(*name)(), uint32_t (*id)()) :
		new_instance(inst),
		get_name(name),
		get_id(id) {}

};

const char *get_subscription_name(const uint32_t link_id);
CanStream *create_subscription_stream(const char *link_name, CanLink *can);

#endif /* CANLINK_SUBSCRIPTION_H_ */
