#ifndef __MD_FUNCTION_H__
#define __MD_FUNCTION_H__

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern  "C" {
#endif

typedef int (*module_fn_t)(int argc, char **);

// Modules Function
int ipc_main(int argc, char *argv[]);
int notify_main(int argc, char *argv[]);
int flashfs_main(int argc, char *argv[]);
int param_main(int argc, char *argv[]);
int canlink_main(int argc, char *argv[]);
int roslink_main(int argc, char *argv[]);

// Main Function
int gps_main(int argc, char *argv[]);
int mag_main(int argc, char *argv[]);
int battery_main(int argc, char *argv[]);
int rng_main(int argc, char *argv[]);

// Level Function
int mavlink_main(int argc, char *argv[]);
int shell_main(int argc, char *argv[]);

int module_func(module_fn_t module_function, const char *arguments);
#ifdef __cplusplus
}
#endif
#endif