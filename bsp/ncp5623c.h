#ifndef NCP5623C_H
#define NCP5623C_H

#ifdef __cplusplus
extern "C"{
#endif

#include "cmsis_os.h"
#include "datatypes.h"

#define NCP5623C_I2C_ADDR       0x39

#define NCP5623_LED_CURRENT	0x20	/**< Current register */
#define NCP5623_LED_PWM0	0x40	/**< pwm0 register */
#define NCP5623_LED_PWM1	0x60	/**< pwm1 register */
#define NCP5623_LED_PWM2	0x80	/**< pwm2 register */

#define NCP5623_LED_BRIGHT	0x1f	/**< full brightness */
#define NCP5623_LED_OFF		0x00	/**< off */

struct ncp5623_data {
      uint8_t		r;
      uint8_t		g;
      uint8_t		b;
      float             brightness;
};

uint8_t ncp5623_init(void);
uint8_t ncp5623_write(struct ncp5623_data *raw);

#ifdef __cplusplus
}
#endif

#endif
