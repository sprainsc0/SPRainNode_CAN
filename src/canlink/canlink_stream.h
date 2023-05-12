#ifndef CANLINK_STREAM_H_
#define CANLINK_STREAM_H_

#include <hrt_timer.h>

class CanLink;

class CanStream
{

public:
	CanStream *next;

	CanStream(CanLink *can);
	virtual ~CanStream();

	/**
	 * Get the interval
	 *
	 * @param interval the interval in microseconds (us) between messages
	 */
	void set_interval(const int interval);

	/**
	 * Get the interval
	 *
	 * @return the inveral in microseconds (us) between messages
	 */
	int get_interval() { return _interval; }

	/**
	 * @return 0 if updated / sent, -1 if unchanged
	 */
	int update(const uint64_t t, const uint8_t index = 0, const void *data = nullptr);
	virtual const char *get_name() const = 0;
	virtual uint32_t get_id() = 0;
	virtual uint32_t Subscription() = 0;

protected:
	CanLink *_can;
	int _interval;		///< if set to negative value = unlimited rate

	virtual bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr) = 0;

private:
	uint64_t _last_write;

	/* do not allow top copying this class */
	CanStream(const CanStream &);
	CanStream &operator=(const CanStream &);
};


#endif /* MAVLINK_STREAM_H_ */
