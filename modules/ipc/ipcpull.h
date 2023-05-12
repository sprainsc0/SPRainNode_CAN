#ifndef IPC_PULL_H_
#define IPC_PULL_H_

#include <utlist.h>
#include <ipccore.h>
#include <hrt_timer.h>

class IPCPull
{
public:
	IPCPull *next;
	
	IPCPull(const ipc_id_t topic);
	~IPCPull();

	/**
	 * Copy topic data to given buffer.
	 *
	 * @return true only if topic data copied successfully.
	 */
	bool update(void *data);

	/**
	 * Check if the subscription has been updated.
	 *
	 * @return true if there has been an update which has been
	 * copied successfully.
	 */
	bool update_if_changed(void *data);

	/**
	 * Check if the topic has been published.
	 *
	 * This call will return true if the topic was ever published.
	 * @return true if the topic has been published at least once.
	 * If no data is available the buffer will be filled with zeros.
	 */
	bool is_published();

	ipc_id_t get_topic() const;

	int get_fd() { return _sfd; }

private:
	const ipc_id_t _topic;		///< topic metadata
	int _sfd;			///< subscription handle
	bool _published;		///< topic was ever published
	uint64_t _last_pub_check;	///< when we checked last

	/* do not allow copying this class */
	IPCPull(const IPCPull &);
	IPCPull operator=(const IPCPull &);
};


#endif /* IPC_PULL_H_ */
