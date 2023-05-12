#pragma once

#include "ipccore.h"

#define LED_PATTERN_BGC_OFF             0x0000      /**< turn off led */
#define LED_PATTERN_BGC_ON              0xFFFF      /**< turn on led */
#define LED_PATTERN_BGC_SLOW    		0x0003		/**< slow blinking */
#define LED_PATTERN_BGC_FAST        	0x5555		/**< fast blinking */
#define LED_PATTERN_BGC_MOD1 			0x0500		/**< long off, then quad blink */
#define LED_PATTERN_BGC_MOD2 			0x0550
#define LED_PATTERN_BGC_MOD3 			0x050F

#define NOTIFY_LOCK                     (0x01)
#define NOTIFY_UNLOCK                   (0x00)

#define NOTIFY_PRIORITY_VERY_HIGH       (0x80)
#define NOTIFY_PRIORITY_HIGH            (0x40)
#define NOTIFY_PRIORITY_MEDIUM          (0x20)
#define NOTIFY_PRIORITY_LOW             (0x10)
#define NOTIFY_PRIORITY_VERY_LOW        (0x08)

#define NOTIFY_GROUP_CAL                (0x01)
#define NOTIFY_GROUP_CHECK              (0x00)

#define NOTIFY_RGB_RED                  (0xFF0000)
#define NOTIFY_RGB_GREEN                (0x00FF00)
#define NOTIFY_RGB_BLUE                 (0x0000FF)

struct actuator_notify_s {
	uint64_t timestamp; // required for logger
	uint8_t lock;
	uint8_t group;
	uint8_t priority;
	uint16_t led_status;
	uint32_t rgb;
#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
IPC_DECLARE(actuator_notify);

