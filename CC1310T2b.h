

#ifndef __CC1310_LAUNCHXL_BOARD_H__
#define __CC1310_LAUNCHXL_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>

extern const PIN_Config BoardGpioInitTable[];

#define THREADSTACKSIZE    1024  /* Stack size in bytes */
#define TASKSTACKSIZE      1024

#define Board_ADCBUF0      CC1310_LAUNCHXL_ADCBUF0

/* I2C */
#define CC1310_LAUNCHXL_I2C0_SCL0             IOID_4
#define CC1310_LAUNCHXL_I2C0_SDA0             IOID_5

/* SPI */
#define SPI_FLASH_CS                          IOID_18

/* SPI Board */
#define CC1310_LAUNCHXL_SPI0_MISO              PIN_UNASSIGNED//IOID_8          /* RF1.20 */
#define CC1310_LAUNCHXL_SPI0_MOSI             PIN_UNASSIGNED//IOID_9          /* RF1.18 */
#define CC1310_LAUNCHXL_SPI0_CLK              IOID_10         /* RF1.16 */
#define CC1310_LAUNCHXL_SPI0_CSN              IOID_11
#define CC1310_LAUNCHXL_SPI1_MISO             PIN_UNASSIGNED
#define CC1310_LAUNCHXL_SPI1_MOSI             PIN_UNASSIGNED
#define CC1310_LAUNCHXL_SPI1_CLK              PIN_UNASSIGNED
#define CC1310_LAUNCHXL_SPI1_CSN              PIN_UNASSIGNED


void CC1310_initGeneral(void);
void CC1310_LAUNCHXL_shutDownExtFlash(void);
void CC1310_LAUNCHXL_wakeUpExtFlash(void);

/*!
 *  @def    CC1310_LAUNCHXL_ADCBufName
 *  @brief  Enum of ADCBufs
 */
typedef enum CC1310_LAUNCHXL_ADCBufName {
    CC1310_LAUNCHXL_ADCBUF0 = 0,

    CC1310_LAUNCHXL_ADCBUFCOUNT
} CC1310_LAUNCHXL_ADCBufName;

/*!
 *  @def    CC1310_LAUNCHXL_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC1310_LAUNCHXL_ADCBuf0ChannelName {
    ADCBUF0CHANNEL0 = 0,
    ADCBUF0CHANNEL1,
    ADCBUF0_CURRENT,       //DIO25 6 нога сверху
    ADCBUF0CHANNELVDDS,
    ADCBUF0CHANNELDCOUPL,
    ADCBUF0CHANNELVSS,

    CC1310_LAUNCHXL_ADCBUF0CHANNELCOUNT
} CC1310_LAUNCHXL_ADCBuf0ChannelName;

/*!
 *  @def    CC1310_LAUNCHXL_ADCName
 *  @brief  Enum of ADCs
*/
typedef enum CC1310_LAUNCHXL_ADCName {
    CC1310_LAUNCHXL_ADC0 = 0,
    CC1310_LAUNCHXL_ADC1,
    CC1310_LAUNCHXL_ADC2,
    CC1310_LAUNCHXL_ADCVSS,
    CC1310_LAUNCHXL_ADCVDDS,

    CC1310_LAUNCHXL_ADCCOUNT
} CC1310_LAUNCHXL_ADCName;

//GPIO Names (к порядку PIN_Config не имеет отношения)
typedef enum CC1310_LAUNCHXL_GPIOName {
    LED1,
    LED2,
    D_OUT,
    PD_SCK,
    //IRQ_ZEROCROSS,
    BUTTON1,
    //ENABLE_4V,
    //GSM_PWR_KEY,
    //GSM_CTS,

    //GSM_UART_RX,
    GSM_UART_TX,
//    SPI0_MOSI,
//    SPI0_MISO,
//    SPI0_CLK,

    CC1310_GPIOCOUNT
} CC1310_LAUNCHXL_GPIOName;

/*!
 *  @def    CC1310_LAUNCHXL_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC1310_LAUNCHXL_GPTimerName {
    CC1310_LAUNCHXL_GPTIMER0A = 0,
    CC1310_LAUNCHXL_GPTIMER0B,
    CC1310_LAUNCHXL_GPTIMER1A,
    CC1310_LAUNCHXL_GPTIMER1B,
    CC1310_LAUNCHXL_GPTIMER2A,
    CC1310_LAUNCHXL_GPTIMER2B,
    CC1310_LAUNCHXL_GPTIMER3A,
    CC1310_LAUNCHXL_GPTIMER3B,

    CC1310_LAUNCHXL_GPTIMERPARTSCOUNT
} CC1310_LAUNCHXL_GPTimerName;

/*!
 *  @def    CC1310_LAUNCHXL_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC1310_LAUNCHXL_GPTimers {
    CC1310_LAUNCHXL_GPTIMER0 = 0,
    CC1310_LAUNCHXL_GPTIMER1,
    CC1310_LAUNCHXL_GPTIMER2,
    CC1310_LAUNCHXL_GPTIMER3,

    CC1310_LAUNCHXL_GPTIMERCOUNT
} CC1310_LAUNCHXL_GPTimers;

/*!
 *  @def    CC1310_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC1310_LAUNCHXL_I2CName {
    CC1310_LAUNCHXL_I2C0 = 0,

    CC1310_LAUNCHXL_I2CCOUNT
} CC1310_LAUNCHXL_I2CName;


/*!
 *  @def    CC1310_LAUNCHXL_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC1310_LAUNCHXL_NVSName {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    CC1310_LAUNCHXL_NVSCC26XX0 = 0,
#endif
#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH
    CC1310_LAUNCHXL_NVSSPI25X0,
#endif

    CC1310_LAUNCHXL_NVSCOUNT
} CC1310_LAUNCHXL_NVSName;

/*!
 *  @def    CC1310_LAUNCHXL_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC1310_LAUNCHXL_SPIName {
    CC1310_LAUNCHXL_SPI0 = 0,

    CC1310_LAUNCHXL_SPICOUNT
} CC1310_LAUNCHXL_SPIName;

/*!
 *  @def    CC1310_LAUNCHXL_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC1310_LAUNCHXL_UARTName {
    CC1310_LAUNCHXL_UART0 = 0,

    CC1310_LAUNCHXL_UARTCOUNT
} CC1310_LAUNCHXL_UARTName;

/*!
 *  @def    CC1310_LAUNCHXL_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1310_LAUNCHXL_UDMAName {
    CC1310_LAUNCHXL_UDMA0 = 0,

    CC1310_LAUNCHXL_UDMACOUNT
} CC1310_LAUNCHXL_UDMAName;

/*!
 *  @def    CC1310_LAUNCHXL_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC1310_LAUNCHXL_WatchdogName {
    CC1310_LAUNCHXL_WATCHDOG0 = 0,

    CC1310_LAUNCHXL_WATCHDOGCOUNT
} CC1310_LAUNCHXL_WatchdogName;

//void I_TimeoutCb(GPTimerCC26XX_Handle handle_I,
//                 GPTimerCC26XX_IntMask interruptMask_I);


#ifdef __cplusplus
}
#endif

#endif /* __CC1310_LAUNCHXL_BOARD_H__ */
