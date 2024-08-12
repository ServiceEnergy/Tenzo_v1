
#include <CC1310T2b.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/udma.h>
#include <ti/devices/cc13x0/inc/hw_ints.h>
#include <ti/devices/cc13x0/inc/hw_memmap.h>

/*
 *  =============================== ADCBuf ===============================
 */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>

ADCBufCC26XX_Object adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBUFCOUNT];

/*
 *  This table converts a virtual adc channel into a dio and internal analogue
 *  input signal. This table is necessary for the functioning of the adcBuf
 *  driver. Comment out unused entries to save flash. Dio and internal signal
 *  pairs are hardwired. Do not remap them in the table. You may reorder entire
 *  entries. The mapping of dio and internal signals is package dependent.
 */
const ADCBufCC26XX_AdcChannelLutEntry ADCBufCC26XX_adcChannelLut[CC1310_LAUNCHXL_ADCBUF0CHANNELCOUNT] = {
    {IOID_23, ADC_COMPB_IN_AUXIO7},   //ADC_COMPB_IN_AUXIO7 (0x09)
    {IOID_24, ADC_COMPB_IN_AUXIO6},   //Не понятно, что это за адреса см. rom.h
    {IOID_25, ADC_COMPB_IN_AUXIO5},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
};

const ADCBufCC26XX_HWAttrs adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBUFCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .adcChannelLut     = ADCBufCC26XX_adcChannelLut,
    }
};

const ADCBuf_Config ADCBuf_config[CC1310_LAUNCHXL_ADCBUFCOUNT] = {
    {
        &ADCBufCC26XX_fxnTable,
        &adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBUF0],
        &adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = CC1310_LAUNCHXL_ADCBUFCOUNT;


/*
 *  =============================== ADC ===============================
*/
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

ADCCC26XX_Object adcCC26xxObjects[CC1310_LAUNCHXL_ADCCOUNT];


const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCCOUNT] = {
    {
        .adcDIO              = IOID_23,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_170_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = IOID_24,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO6,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_170_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = IOID_25,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO5,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_170_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VSS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_170_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VDDS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_170_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    }

};

const ADC_Config ADC_config[CC1310_LAUNCHXL_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC0], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC0]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC1], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC1]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC2], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC2]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCVSS], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCVSS]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCVDDS], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCVDDS]},
};

const uint_least8_t ADC_count = CC1310_LAUNCHXL_ADCCOUNT;


/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

// Опрределяем псевдонимы пинов в файле СС1310T2b.h в том же порядке что и здесь
// Только для физических пинов, которые не задействованы в драйверах
GPIO_PinConfig gpioPinConfigs[] = {
   GPIOCC26XX_DIO_01 |  GPIO_CFG_OUTPUT | GPIO_CFG_OUT_STD |GPIO_CFG_OUT_LOW,   // LED1 (The same as LanchBoard)
   GPIOCC26XX_DIO_02 | GPIO_CFG_OUTPUT | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,   // LED2 (The same as LanchBoard)
   GPIOCC26XX_DIO_03 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,  /* D_OUT HX711*/
   GPIOCC26XX_DIO_04 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_LOW | GPIO_CFG_OUT_LOW,  /* PD_SCK */
   // GPIOCC26XX_DIO_01 | GPIO_DO_NOT_CONFIG,   // AXEL_POWER Axelerometer Power
    GPIOCC26XX_DIO_13 | GPIO_DO_NOT_CONFIG,   // IRQ_ZEROCROSS IRQ IOID13 (16)
    GPIOCC26XX_DIO_14 | GPIO_DO_NOT_CONFIG,   // BUTTON1 is active low  (The same as LanchBoard)
    GPIOCC26XX_DIO_30 | GPIO_DO_NOT_CONFIG,   // ENABLE_4V
    GPIOCC26XX_DIO_22 | GPIO_DO_NOT_CONFIG,   // GSM_PWR_KEY
    //GPIOCC26XX_DIO_01 | GPIO_DO_NOT_CONFIG,   // GSM_CTS
     //GPIOCC26XX_DIO_02 | GPIO_DO_NOT_CONFIG,   // GSM_UART_RX
     //GPIOCC26XX_DIO_03 | GPIO_DO_NOT_CONFIG,   // GSM_UART_TX
     //GPIOCC26XX_DIO_04 | GPIO_DO_NOT_CONFIG,   // SPI0_MOSI
    // GPIOCC26XX_DIO_10 | GPIO_DO_NOT_CONFIG,   // SPI0_MISO
    // GPIOCC26XX_DIO_08 | GPIO_DO_NOT_CONFIG,   // SPI0_CLK
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1310_LAUNCH.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /* Button 0 */
    NULL,  /* Button 1 */
    NULL,  /* CC1310_LAUNCHXL_SPI_MASTER_READY */
    NULL,  /* CC1310_LAUNCHXL_SPI_SLAVE_READY */
};

const GPIOCC26XX_Config GPIOCC26XX_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = CC1310_GPIOCOUNT,
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[CC1310_LAUNCHXL_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER0], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER0], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER0B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER1], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER1], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER2], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER2], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER3], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER3], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER3B], GPT_B },
};

/*
 *  =============================== I2C ===============================
*/
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

I2CCC26XX_Object i2cCC26xxObjects[CC1310_LAUNCHXL_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC1310_LAUNCHXL_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = CC1310_LAUNCHXL_I2C0_SDA0,
        .sclPin      = CC1310_LAUNCHXL_I2C0_SCL0,
    }
};

const I2C_Config I2C_config[CC1310_LAUNCHXL_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
       .object      = &i2cCC26xxObjects[CC1310_LAUNCHXL_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CC1310_LAUNCHXL_I2C0]
    }
};

const uint_least8_t I2C_count = CC1310_LAUNCHXL_I2CCOUNT;


/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSSPI25X.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#define NVS_REGIONS_BASE 0x1A000
#define SECTORSIZE       0x1000
#define REGIONSIZE       (SECTORSIZE * 4)

/*
 * Reserve flash sectors for NVS driver use by placing an uninitialized byte
 * array at the desired flash address.
 */
#if defined(__TI_COMPILER_VERSION__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
#pragma LOCATION(flashBuf, NVS_REGIONS_BASE);
#pragma NOINIT(flashBuf);
static char flashBuf[REGIONSIZE];

/* Allocate objects for NVS Internal Regions */
NVSCC26XX_Object nvsCC26xxObjects[1];

/* Hardware attributes for NVS Internal Regions */
const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[1] = {
    {
        .regionBase = (void *)flashBuf,
        .regionSize = REGIONSIZE,
    },
};

#endif /* Board_EXCLUDE_NVS_INTERNAL_FLASH */

#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH

#define SPISECTORSIZE    0x1000
#define SPIREGIONSIZE    (SPISECTORSIZE * 32)
#define VERIFYBUFSIZE    64

static uint8_t verifyBuf[VERIFYBUFSIZE];

/* Allocate objects for NVS External Regions */
NVSSPI25X_Object nvsSPI25XObjects[1];

/* Hardware attributes for NVS External Regions */
const NVSSPI25X_HWAttrs nvsSPI25XHWAttrs[1] = {
    {
        .regionBaseOffset = 0,
        .regionSize = SPIREGIONSIZE,
        .sectorSize = SPISECTORSIZE,
        .verifyBuf = verifyBuf,
        .verifyBufSize = VERIFYBUFSIZE,
        .spiHandle = NULL,
        .spiIndex = 0,
        .spiBitRate = 4000000,
        .spiCsnGpioIndex = SPI_FLASH_CS,
        .statusPollDelayUs = 100,
    },
};

#endif /* Board_EXCLUDE_NVS_EXTERNAL_FLASH */

/* NVS Region index 0 and 1 refer to NVS and NVS SPI respectively */
const NVS_Config NVS_config[CC1310_LAUNCHXL_NVSCOUNT] = {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26xxObjects[0],
        .hwAttrs = &nvsCC26xxHWAttrs[0],
    },
#endif
#ifndef Board_EXCLUDE_NVS_EXTERNAL_FLASH
    {
        .fxnTablePtr = &NVSSPI25X_fxnTable,
        .object = &nvsSPI25XObjects[0],
        .hwAttrs = &nvsSPI25XHWAttrs[0],
    },
#endif
};

const uint_least8_t NVS_count = CC1310_LAUNCHXL_NVSCOUNT;

/*
 *  =============================== PIN ===============================
 */
//В том же порядке что и GPIO, хотя это и не обязательно
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

const PIN_Config BoardGpioInitTable[] = {
  IOID_1   | PIN_GPIO_OUTPUT_EN |PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MIN,    // LED1 (The same as LanchBoard)
  IOID_2   | PIN_GPIO_OUTPUT_EN |PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MIN,    // LED1 (The same as LanchBoard)
  IOID_3   | PIN_INPUT_EN |PIN_PULLUP  | PIN_IRQ_NEGEDGE,       // D_OUT (HX711)
  IOID_4   | PIN_GPIO_OUTPUT_EN |PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MIN,    // PD_SCK (HX711)
 // IOID_13  | PIN_INPUT_EN       |PIN_PULLUP    | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,  // IRQ  IOID13 (16) см. pin.h
  IOID_19  | PIN_INPUT_EN       |PIN_PULLUP    | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,  // Button is active low  (The same as LanchBoard)
 // IOID_18  | PIN_GPIO_OUTPUT_EN |PIN_GPIO_HIGH | PIN_PUSHPULL      | PIN_DRVSTR_MIN,  // SPI_FLASH_CS
 // IOID_30  | PIN_GPIO_OUTPUT_EN |PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MIN,  // Enable_4V
 // IOID_22  | PIN_GPIO_OUTPUT_EN |PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MIN,  // GSM_PWR_KEY
  //IOID_21  | PIN_INPUT_EN       |PIN_PULLDOWN,                                        // GSM_UART_RX
  IOID_20  | PIN_GPIO_OUTPUT_EN |PIN_GPIO_HIGH | PIN_PUSHPULL,                        // GSM_UART_TX
 // GSM_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* UART RX via debugger back channel */
  //GSM_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,                        /* UART TX via debugger back channel */
 // IOID_9   | PIN_INPUT_EN       |PIN_PULLDOWN,                                        // SPI0_MOSI
 // IOID_10  | PIN_INPUT_EN       |PIN_PULLDOWN,                                        // SPI0_MISO
 // IOID_8   | PIN_INPUT_EN       |PIN_PULLDOWN,                                        // SPI0_CLK
  PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = &PowerCC26XX_standbyPolicy,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = true,
    .calibrateRCOSC_LF  = true,
    .calibrateRCOSC_HF  = true,
};

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,       /* Lowest HWI priority */
    .swiPriority        = 0,        /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,     /* Keep XOSC dependency while in standby */
    .globalCallback     = NULL,     /* No board specific callback */
    .globalEventMask    = 0         /* No events subscribed to */
};

/*
 *  =============================== SPI DMA ===============================
 */
//#include <ti/drivers/SPI.h>
//#include <ti/drivers/spi/SPICC26XXDMA.h>

//SPICC26XXDMA_Object spiCC26XXDMAObjects[CC1310_LAUNCHXL_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
//const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC1310_LAUNCHXL_SPICOUNT] = {
//    {
//        .baseAddr           = SSI0_BASE,
//        .intNum             = INT_SSI0_COMB,
//        .intPriority        = ~0,
//        .swiPriority        = 0,
//        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
//        .defaultTxBufValue  = 0xFF,
//        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
//        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
//        .mosiPin            = CC1310_LAUNCHXL_SPI0_MOSI,
//        .misoPin            = CC1310_LAUNCHXL_SPI0_MISO,
//        .clkPin             = CC1310_LAUNCHXL_SPI0_CLK,
//        .csnPin             = CC1310_LAUNCHXL_SPI0_CSN,
//        .minDmaTransferSize = 10
//    },
//};
//
//const SPI_Config SPI_config[CC1310_LAUNCHXL_SPICOUNT] = {
//    {
//         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
//         .object      = &spiCC26XXDMAObjects[CC1310_LAUNCHXL_SPI0],
//         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1310_LAUNCHXL_SPI0]
//    },
//};
//
//const uint_least8_t SPI_count = CC1310_LAUNCHXL_SPICOUNT;
//

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects[CC1310_LAUNCHXL_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CC1310_LAUNCHXL_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CC1310_LAUNCHXL_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = IOID_20, // GSM_UART_TX В драйверах - физический PIN а не GPIOName!!!
        .rxPin          = IOID_21, // GSM_UART_RX В драйверах - физический PIN а не GPIOName!!!
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[CC1310_LAUNCHXL_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[CC1310_LAUNCHXL_UART0]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    }
};

const UART_Config UART_config[CC1310_LAUNCHXL_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CC1310_LAUNCHXL_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[CC1310_LAUNCHXL_UART0]
    },
};

const uint_least8_t UART_count = CC1310_LAUNCHXL_UARTCOUNT;

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC1310_LAUNCHXL_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC1310_LAUNCHXL_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC1310_LAUNCHXL_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC1310_LAUNCHXL_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC1310_LAUNCHXL_UDMA0]
    },
};

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC1310_LAUNCHXL_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC1310_LAUNCHXL_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 1000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[CC1310_LAUNCHXL_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[CC1310_LAUNCHXL_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[CC1310_LAUNCHXL_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = CC1310_LAUNCHXL_WATCHDOGCOUNT;


/*
 *  Board-specific initialization function to disable external flash.
 *  This function is defined in the file CC1310_LAUNCHXL_fxns.c
 */
extern void Board_initHook(void);

/*
 *  ======== CC1310_LAUNCHXL_initGeneral ========
 */
void CC1310_initGeneral(void)
{
    Power_init();

    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        GPIO_toggle(LED2);
        while (1);
    }

    /* Perform board-specific initialization */
    Board_initHook();
}


