#include <string>
#include "ipccore.h"
#include "debug.h"
#include "buildin_function.h"

int ipc_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error\n");
		return 1;
	}
    for(int i=0; i<argc; i++) {
        if (!strcmp(argv[i], "start")) {
            ipc_init();
        }
    }
    return 1;
}

