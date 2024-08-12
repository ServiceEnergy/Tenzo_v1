/*
 * LSM6DSL.h
 *
 *  Created on: 13 march 2022 ã.
 *      Author: Tokarev V.A.
 */
#include <stdint.h>
#include <stdbool.h>

#define SAD            0b1101010   // Slave address of LSM6DSL

// Registers
#define CTRL1_XL        0x10
#define OUTX_L_XL       0x28
#define OUTX_L_G        0x22
#define OUT_TEMP_L      0x20
#define OUT_TEMP_H      0x21
#define CTRL6_C         0x15
#define CTRL3_Ñ         0x12
#define CTRL2_G         0x11
#define INT1_CTRL       0x0D
#define STATUS_REG      0x1E
#define CTRL7_G         0x16
#define CTRL5_C         0x14
#define CTRL8_XL        0x17
#define FIFO_CTRL1      0x06
#define FIFO_CTRL2      0x07
#define FIFO_CTRL3      0x08
#define FIFO_CTRL4      0x09
#define FIFO_CTRL5      0x0A
#define FIFO_STATUS1    0x3A
#define FIFO_STATUS2    0x3B
#define FIFO_STATUS3    0x3C
#define FIFO_STATUS4    0x3D

#define FIFO_DATA_OUTL  0x3E

#define XL_HM_MODE      4          //  high-performance operating mode disabled
#define INT1_DRDY_G     1          //  Gyroscope Data Ready on INT1 pad
#define INT1_DRDY_XL    0          //  Accelerometer Data Ready on INT1 pad
#define EN_XL_1_6       0xB0       //  Enable accelerometer 1.6 Hz (low power only) +-2g
#define EN_XL_52        0x30       //  Enable accelerometer 52 Hz  (low power) +-2g
#define EN_XL_833       0x70       //  Enable accelerometer 833 Hz (high performance) +-2g
#define EN_XL_208       0x50       //  Enable accelerometer 208 Hz (normal performance) +-2g
#define EN_G_208        0x50       //  Enable gyroscope 208 Hz 245 dps
#define HP_EN_G         6          //  Gyroscope digital high-pass filter enable

bool Write_Byte(uint8_t addr, uint8_t data);
uint8_t read_Byte(uint8_t addr);
bool bool_read_Byte(uint8_t addr,uint8_t *byte);
bool LSM6_Read_XL (int16_t *x, int16_t *y, int16_t *z);
bool LSM6_Read_G (int16_t *x, int16_t *y, int16_t *z);
bool LSM6_Read_G_XL (int16_t *xg, int16_t *yg, int16_t *zg, int16_t *xxl, int16_t *yxl, int16_t *zxl);
bool LSM6_Read_Temp (int16_t *t);
//bool TC74_Read_Temp (int8_t *t);
//float DS1621_Read_Temp (void);
bool LM75_Read_Temp (float *t);
bool LM75_on(void);
bool LM75_off(void);
bool LSM6_FIFO_Read(int16_t *x);
bool LSM6_FIFO_Read_complex(uint8_t *b, uint8_t *b1,int16_t *x);
bool LSM6_FIFO_Read_complex3(uint8_t *b, uint8_t *b1,int16_t *x,int16_t *y,int16_t *z);
//bool LSM6_FIFO_Read3(int16_t *x,int16_t *y,int16_t *z);
//bool LSM6_FIFO_Read6(int16_t *xg, int16_t *yg, int16_t *zg,int16_t *x,int16_t *y,int16_t *z);

#ifndef LSM6DSL_H_
#define LSM6DSL_H_





#endif /* LSM6DSL_H_ */
