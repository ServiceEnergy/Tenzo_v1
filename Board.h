

#ifndef __BOARD_H
#define __BOARD_H

#define Board_CC1310_LAUNCHXL

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Board.h>

//#define Board_initGeneral()     Board_init()  /* deprecated */

#include <CC1310T2b.h>

#define Board_shutDownExtFlash() CC1310_LAUNCHXL_shutDownExtFlash()
#define Board_wakeUpExtFlash() CC1310_LAUNCHXL_wakeUpExtFlash()

/* These #defines allow us to reuse TI-RTOS across other device families */
/********
#define Board_ADC0              CC1310_LAUNCHXL_ADC0
#define Board_ADC1              CC1310_LAUNCHXL_ADC1



#define Board_CRYPTO0           CC1310_LAUNCHXL_CRYPTO0
#define Board_AESCCM0           CC1310_LAUNCHXL_AESCCM0
#define Board_AESGCM0           CC1310_LAUNCHXL_AESGCM0
#define Board_AESCBC0           CC1310_LAUNCHXL_AESCBC0
#define Board_AESCTR0           CC1310_LAUNCHXL_AESCTR0
#define Board_AESECB0           CC1310_LAUNCHXL_AESECB0
#define Board_AESCTRDRBG0       CC1310_LAUNCHXL_AESCTRDRBG0
#define Board_TRNG0             CC1310_LAUNCHXL_TRNG0

#define Board_DIO0              CC1310_LAUNCHXL_DIO0
#define Board_DIO1              CC1310_LAUNCHXL_DIO1
#define Board_DIO12             CC1310_LAUNCHXL_DIO12
#define Board_DIO15             CC1310_LAUNCHXL_DIO15
#define Board_DIO16_TDO         CC1310_LAUNCHXL_DIO16_TDO
#define Board_DIO17_TDI         CC1310_LAUNCHXL_DIO17_TDI
#define Board_DIO21             CC1310_LAUNCHXL_DIO21
#define Board_DIO22             CC1310_LAUNCHXL_DIO22

#define Board_GPIO_BUTTON0      CC1310_LAUNCHXL_GPIO_S1
#define Board_GPIO_BUTTON1      CC1310_LAUNCHXL_GPIO_S2
#define Board_GPIO_BTN1         CC1310_LAUNCHXL_GPIO_S1
#define Board_GPIO_BTN2         CC1310_LAUNCHXL_GPIO_S2
#define Board_GPIO_LED0         CC1310_LAUNCHXL_GPIO_LED_RED
#define Board_GPIO_LED1         CC1310_LAUNCHXL_GPIO_LED_GREEN
#define Board_GPIO_RLED         CC1310_LAUNCHXL_GPIO_LED_RED
#define Board_GPIO_GLED         CC1310_LAUNCHXL_GPIO_LED_GREEN
#define Board_GPIO_LED_ON       CC1310_LAUNCHXL_GPIO_LED_ON
#define Board_GPIO_LED_OFF      CC1310_LAUNCHXL_GPIO_LED_OFF
#define Board_GPIO_TMP116_EN    CC1310_LAUNCHXL_GPIO_TMP116_EN

#define Board_GPTIMER0A         CC1310_LAUNCHXL_GPTIMER0A
#define Board_GPTIMER0B         CC1310_LAUNCHXL_GPTIMER0B
#define Board_GPTIMER1A         CC1310_LAUNCHXL_GPTIMER1A
#define Board_GPTIMER1B         CC1310_LAUNCHXL_GPTIMER1B
#define Board_GPTIMER2A         CC1310_LAUNCHXL_GPTIMER2A
#define Board_GPTIMER2B         CC1310_LAUNCHXL_GPTIMER2B
#define Board_GPTIMER3A         CC1310_LAUNCHXL_GPTIMER3A
#define Board_GPTIMER3B         CC1310_LAUNCHXL_GPTIMER3B

#define Board_I2C0              CC1310_LAUNCHXL_I2C0
#define Board_I2C_TMP           CC1310_LAUNCHXL_I2C0

#define Board_I2S0              CC1310_LAUNCHXL_I2S0
#define Board_I2S_ADO           CC1310_LAUNCHXL_I2S_ADO
#define Board_I2S_ADI           CC1310_LAUNCHXL_I2S_ADI
#define Board_I2S_BCLK          CC1310_LAUNCHXL_I2S_BCLK
#define Board_I2S_MCLK          CC1310_LAUNCHXL_I2S_MCLK
#define Board_I2S_WCLK          CC1310_LAUNCHXL_I2S_WCLK

#define Board_NVSINTERNAL       CC1310_LAUNCHXL_NVSCC26XX0
#define Board_NVSEXTERNAL       CC1310_LAUNCHXL_NVSSPI25X0

#define Board_PIN_BUTTON0       CC1310_LAUNCHXL_PIN_BTN1
#define Board_PIN_BUTTON1       CC1310_LAUNCHXL_PIN_BTN2
#define Board_PIN_BTN1          CC1310_LAUNCHXL_PIN_BTN1
#define Board_PIN_BTN2          CC1310_LAUNCHXL_PIN_BTN2
#define Board_PIN_LED0          CC1310_LAUNCHXL_PIN_RLED
#define Board_PIN_LED1          CC1310_LAUNCHXL_PIN_GLED
#define Board_PIN_LED2          CC1310_LAUNCHXL_PIN_RLED
#define Board_PIN_RLED          CC1310_LAUNCHXL_PIN_RLED
#define Board_PIN_GLED          CC1310_LAUNCHXL_PIN_GLED

#define Board_PWM0              CC1310_LAUNCHXL_PWM0
#define Board_PWM1              CC1310_LAUNCHXL_PWM1
#define Board_PWM2              CC1310_LAUNCHXL_PWM2
#define Board_PWM3              CC1310_LAUNCHXL_PWM3
#define Board_PWM4              CC1310_LAUNCHXL_PWM4
#define Board_PWM5              CC1310_LAUNCHXL_PWM5
#define Board_PWM6              CC1310_LAUNCHXL_PWM6
#define Board_PWM7              CC1310_LAUNCHXL_PWM7

#define Board_SD0               CC1310_LAUNCHXL_SDSPI0

#define Board_SPI0              CC1310_LAUNCHXL_SPI0
#define Board_SPI1              CC1310_LAUNCHXL_SPI1
#define Board_SPI_FLASH_CS      CC1310_LAUNCHXL_SPI_FLASH_CS
#define Board_FLASH_CS_ON       0
#define Board_FLASH_CS_OFF      1

#define Board_SPI_MASTER        CC1310_LAUNCHXL_SPI0
#define Board_SPI_SLAVE         CC1310_LAUNCHXL_SPI0
#define Board_SPI_MASTER_READY  CC1310_LAUNCHXL_SPI_MASTER_READY
#define Board_SPI_SLAVE_READY   CC1310_LAUNCHXL_SPI_SLAVE_READY

#define Board_UART0             CC1310_LAUNCHXL_UART0

#define Board_WATCHDOG0         CC1310_LAUNCHXL_WATCHDOG0
***********/

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
