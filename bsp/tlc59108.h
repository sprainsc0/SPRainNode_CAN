#ifndef TLC59108_H
#define TLC59108_H

#ifdef __cplusplus
extern "C"{
#endif

#include "cmsis_os.h"
#include "datatypes.h"

#define TLC59108_I2C_ADDR             0x42

#define AUTO_INCREMENT_ALL            0x80  // increment through all registers (for initial setup)
#define AUTO_INCREMENT_IND            0xA0  // increment through individual brightness registers
#define AUTO_INCREMENT_GLOBAL         0xC0  // increment through global control registers
#define AUTO_INCREMENT_INDGLOBAL      0xE0  // increment through individual and global registers

#define RGBLED0_Regaddr               0x02//00000010
#define RGBLED1_Regaddr               0x03//00000011
#define RGBLED2_Regaddr               0x04//00000100
#define RGBLED3_Regaddr               0x05//00000101
#define RGBLED4_Regaddr               0x06//00000110
#define RGBLED5_Regaddr               0x07//00000111
#define RGBLED6_Regaddr               0x08//00000110
#define RGBLED7_Regaddr               0x09//00000111

#define RGBLED_Mode1Regaddr           0x00
#define RGBLED_Mode2Regaddr           0x01

#define RGBLED_OutStatus0Regaddr      0x0C
#define RGBLED_OutStatus1Regaddr      0x0D

#define RGBLED_LDRx_Off               0x00
#define RGBLED_LDRx_On                0x01
#define RGBLED_LDRx_PWM               0x02
#define RGBLED_LDRx_PWM_GRPPWM        0x03

#define RGBLED_ControlReg             0 //Disable auto-cremented

struct tlc59108_data {
      uint8_t		r;
      uint8_t		g;
      uint8_t		b;
      float             brightness;
};

uint8_t tlc59108_init(void);
uint8_t tlc59108_write(struct tlc59108_data *raw);

#ifdef __cplusplus
}
#endif

#endif
