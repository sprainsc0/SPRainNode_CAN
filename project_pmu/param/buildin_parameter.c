#include "buildin_parameter.h"

#if defined(UAVCAN_NODE_GPS1) || defined(UAVCAN_NODE_GPS2)
const struct bgc_parameters_t foc_parameters = {
    //    NAME             TYPE              VOLA            DEFAULT
    {"POS_OFFX",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"POS_OFFY",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"POS_OFFZ",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"EXT_ROTAT",    PARAM_TYPE_INT32, .volatile_param = 0,.val.i = 0 },

    4
};
#elif defined(UAVCAN_NODE_PMU)
const struct bgc_parameters_t foc_parameters = {
    //    NAME             TYPE              VOLA            DEFAULT
    {"POS_OFFX",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"POS_OFFY",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"POS_OFFZ",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },

    {"RES_OHM",      PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.05f },

    {"RNG_MAX",      PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 6.0f },
    {"RNG_MIN",      PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.1f },

    6
};
#elif defined(UAVCAN_NODE_ROS)
const struct bgc_parameters_t foc_parameters = {
    //    NAME             TYPE              VOLA            DEFAULT
    {"POS_OFFX",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"POS_OFFY",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"POS_OFFZ",     PARAM_TYPE_FLOAT, .volatile_param = 0,.val.f = 0.0f },
    {"EXT_ROTAT",    PARAM_TYPE_INT32, .volatile_param = 0,.val.i = 0 },

    4
};
#endif