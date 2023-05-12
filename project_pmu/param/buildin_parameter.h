#ifndef _BEAST_PARAM_H
#define _BEAST_PARAM_H

#include <stdint.h>
#include "param.h"

#ifdef __cplusplus
extern "C"{
#endif

#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)
struct bgc_parameters_t {
    /* -------------------acc calibration parameter------------------- */
    const struct param_info_s __param__POS_OFFX;
    const struct param_info_s __param__POS_OFFY;
    const struct param_info_s __param__POS_OFFZ;
    const struct param_info_s __param__EXT_ROTAT;
    
	const unsigned int param_count;
};
#elif defined(UAVCAN_NODE_PMU)
struct bgc_parameters_t {
    /* -------------------acc calibration parameter------------------- */
    const struct param_info_s __param__POS_OFFX;
    const struct param_info_s __param__POS_OFFY;
    const struct param_info_s __param__POS_OFFZ;

    const struct param_info_s __param__RES_OHM;
    const struct param_info_s __param__RNG_MAX;
    const struct param_info_s __param__RNG_MIN;
    
	const unsigned int param_count;
};
#elif defined(UAVCAN_NODE_ROS)
struct bgc_parameters_t {
    /* -------------------acc calibration parameter------------------- */
    const struct param_info_s __param__POS_OFFX;
    const struct param_info_s __param__POS_OFFY;
    const struct param_info_s __param__POS_OFFZ;
    const struct param_info_s __param__EXT_ROTAT;
    
	const unsigned int param_count;
};
#endif

extern const struct bgc_parameters_t foc_parameters;

#ifdef __cplusplus
}
#endif
#endif