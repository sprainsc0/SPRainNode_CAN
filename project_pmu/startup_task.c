#include "main.h"
#include "cmsis_os.h"
#include "platform.h"
#include "buildin_function.h"

extern osThreadId_t startupHandle;

int module_func(module_fn_t module_function, const char *arguments)
{
    char *s = strdup(arguments);
    char *args[6];
    uint8_t nargs = 0;
    char *saveptr = NULL;
    
    for (char *tok=strtok_r(s, " ", &saveptr); tok; tok=strtok_r(NULL, " ", &saveptr)) {
        args[nargs++] = tok;
        if (nargs == 5) {
            break;
        }
    }
    args[nargs++] = NULL;
    
    return module_function(nargs, args);
}

void startup_task(void *argument)
{
	platform_init();
    
    module_func(ipc_main,         "start");
    
	if(!module_func(flashfs_main, "start")) {
       module_func(flashfs_main, "erase start");
   }
   if(!module_func(param_main,   "load")) {
       module_func(param_main,   "reset load");
   }

    module_func(notify_main,      "start");
    module_func(shell_main,       "start");
#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)
    module_func(gps_main,         "start");
    module_func(mag_main,         "start");
#endif
    
#ifdef UAVCAN_NODE_PMU
    module_func(battery_main,     "start cali");
    module_func(rng_main,         "start");
#endif

#ifdef UAVCAN_NODE_ROS
    module_func(mavlink_main,     "start");
#endif

    module_func(canlink_main,     "start");
    
	osThreadTerminate(startupHandle);
}
