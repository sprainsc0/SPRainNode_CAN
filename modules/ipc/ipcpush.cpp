#include "ipcpush.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

IPCPush::IPCPush(const ipc_id_t topic) :
	next(nullptr),
	_topic(topic),
	_pfd(nullptr)
{
}

IPCPush::~IPCPush()
{
	if (_pfd != nullptr) {
		ipc_inactive(_pfd);
	}
}

ipc_id_t
IPCPush::get_topic() const
{
	return _topic;
}

bool
IPCPush::push(const void *data)
{
	if (_pfd == nullptr) {
		_pfd = ipc_active(_topic, data);
	}

	if (ipc_push(_topic, _pfd, data) != 1) {
		return false;
	}

	return true;
}
