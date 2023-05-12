#include <stdlib.h>

#include "canlink_stream.h"
#include "canlink.h"

CanStream::CanStream(CanLink *can) :
	next(nullptr),
	_can(can),
	_interval(1000000),
	_last_write(0 /* 0 means unlimited - updates on every iteration */)
{
}

CanStream::~CanStream()
{
}

/**
 * Set messages interval in ms
 */
void
CanStream::set_interval(const int interval)
{
	_interval = interval;
}

/**
 * Update subscriptions and send message if necessary
 */
int
CanStream::update(const uint64_t t, const uint8_t index, const void *data)
{
	if (_last_write == 0) {

		if (link_proc(t, index, data)) {
			_last_write = t;
            return 0;
		}
		return -1;
	}

	if (_last_write > t) {
		return -1;
	}

	int64_t dt = t - _last_write;
	int interval = (_interval > 0) ? _interval : 0;

	if (interval == 0 || (dt >= interval)) {
		// interval expired, send message
		bool write = true;

		write = link_proc(t, index, data);

		if (write) {
			_last_write = ((interval > 0) && ((int64_t)(1.5f * interval) > dt)) ? _last_write + interval : t;
			return 0;
		} else {
			return -1;
		}
	}

	return -1;
}
