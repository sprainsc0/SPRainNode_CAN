#ifndef IPC_PUSH_H_
#define IPC_PUSH_H_

#include <utlist.h>
#include <ipccore.h>
#include <hrt_timer.h>

class IPCPush
{
public:
	IPCPush *next;

	IPCPush(const ipc_id_t topic);
	~IPCPush();

	/**
	 * Push topic data to given buffer.
	 *
	 * @return true only if topic data copied successfully.
	 */
	bool push(const void *data);

	ipc_id_t get_topic() const;

	orb_advert_t get_fd() { return _pfd; }

private:
	const ipc_id_t _topic;		///< topic metadata
	orb_advert_t _pfd;			///< subscription handle
	bool _published;		///< topic was ever published
	uint64_t _last_pub_check;	///< when we checked last

	/* do not allow copying this class */
	IPCPush(const IPCPush &);
	IPCPush operator=(const IPCPush &);
};


#endif /* IPC_PUSH_H_ */
