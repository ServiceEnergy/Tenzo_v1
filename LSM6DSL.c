/*
 * accelerometr functions
 *
 *  Created on: 13 march 2022 ã.
 *      Author: Tokarev V.A.
 */
/* Application header files */
#include "LSM6DSL.h"
#include <ti/drivers/I2C.h>
#include <ti/sysbios/knl/Task.h>

uint8_t tx_buf[2];
uint8_t rx_buf[12];

extern I2C_Handle      i2c;
extern I2C_Params      i2cParams;
extern I2C_Transaction i2cTransaction;

//I2C_Handle      i2c;
//I2C_Params      i2cParams;
//I2C_Transaction i2cTransaction;

bool Write_Byte(uint8_t addr, uint8_t data)
{
    tx_buf[0] = addr;
    tx_buf[1] = data;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 2;
    return I2C_transfer(i2c, &i2cTransaction);
}

bool bool_read_Byte(uint8_t addr,uint8_t *byte)
{
    tx_buf[0] = addr;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 1;

   if(!I2C_transfer(i2c, &i2cTransaction))
   {
       return false;
   }
   else
   {
       *byte=rx_buf[0];
       return true;
   }

}
uint8_t read_Byte(uint8_t addr)
{
    tx_buf[0] = addr;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 1;

    I2C_transfer(i2c, &i2cTransaction);
    return rx_buf[0];
}

bool LSM6_Read_XL (int16_t *x, int16_t *y, int16_t *z)
{
    tx_buf[0] = OUTX_L_XL;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 6;

    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        return false;
    }
    else
    {
        *x = (int16_t)( rx_buf[0] | (rx_buf[1] << 8) );
        *y = (int16_t)( rx_buf[2] | (rx_buf[3] << 8) );
        *z = (int16_t)( rx_buf[4] | (rx_buf[5] << 8) );
        return true;
    }
}


bool LSM6_FIFO_Read (int16_t *x)
{
        tx_buf[0] = FIFO_DATA_OUTL;
        i2cTransaction.slaveAddress = SAD;
        i2cTransaction.writeBuf = tx_buf;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rx_buf;
        i2cTransaction.readCount = 2;
        if (!I2C_transfer(i2c, &i2cTransaction))
            {
                return false;
            }
            else
            {
                *x = (int16_t)( rx_buf[0] | (rx_buf[1] << 8) );
                return true;
            }
}

bool LSM6_FIFO_Read_complex(uint8_t *b, uint8_t *b1,int16_t *x)
{
    tx_buf[0] = FIFO_STATUS3;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 4;
    if (!I2C_transfer(i2c, &i2cTransaction))    return false;
    else
    {
      *b = rx_buf[0] ;
      *b1 = rx_buf[1];
      *x = (int16_t)( rx_buf[2] | (rx_buf[3] << 8) );
      return true;
    }
}

bool LSM6_FIFO_Read_complex3(uint8_t *b, uint8_t *b1,int16_t *x,int16_t *y,int16_t *z)
{
    tx_buf[0] = FIFO_STATUS3;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 8;
    if (!I2C_transfer(i2c, &i2cTransaction))    return false;
    else
    {
      *b = rx_buf[0] ;
      *b1 = rx_buf[1];
      *x = (int16_t)( rx_buf[2] | (rx_buf[3] << 8) );
      *y = (int16_t)( rx_buf[4] | (rx_buf[5] << 8) );
      *z = (int16_t)( rx_buf[6] | (rx_buf[7] << 8) );
      return true;
    }
}

bool LSM6_FIFO_Read6(int16_t *xg, int16_t *yg, int16_t *zg,int16_t *x,int16_t *y,int16_t *z)
{
    tx_buf[0] = FIFO_DATA_OUTL;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 12;
    if (!I2C_transfer(i2c, &i2cTransaction))    return false;
    else
    {

        *xg = (int16_t)( rx_buf[0] | (rx_buf[1] << 8) );
        *yg = (int16_t)( rx_buf[2] | (rx_buf[3] << 8) );
        *zg = (int16_t)( rx_buf[4] | (rx_buf[5] << 8) );
        *x = (int16_t)( rx_buf[6] | (rx_buf[7] << 8) );
        *y = (int16_t)( rx_buf[8] | (rx_buf[9] << 8) );
        *z = (int16_t)( rx_buf[10] | (rx_buf[11] << 8) );
      return true;
    }
}
bool LSM6_Read_G (int16_t *x, int16_t *y, int16_t *z)
{
    tx_buf[0] = OUTX_L_G;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 6;

    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        return false;
    }
    else
    {
        *x = (int16_t)( rx_buf[0] | (rx_buf[1] << 8) );
        *y = (int16_t)( rx_buf[2] | (rx_buf[3] << 8) );
        *z = (int16_t)( rx_buf[4] | (rx_buf[5] << 8) );
        return true;
    }
}

bool LSM6_Read_G_XL (int16_t *xg, int16_t *yg, int16_t *zg, int16_t *xxl, int16_t *yxl, int16_t *zxl)
{
    tx_buf[0] = OUTX_L_G;
    i2cTransaction.slaveAddress = SAD;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 12;

    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        return false;
    }
    else
    {
        *xg = (int16_t)( rx_buf[0] | (rx_buf[1] << 8) );
        *yg = (int16_t)( rx_buf[2] | (rx_buf[3] << 8) );
        *zg = (int16_t)( rx_buf[4] | (rx_buf[5] << 8) );
        *xxl = (int16_t)( rx_buf[6] | (rx_buf[7] << 8) );
        *yxl = (int16_t)( rx_buf[8] | (rx_buf[9] << 8) );
        *zxl = (int16_t)( rx_buf[10] | (rx_buf[11] << 8) );
        return true;
    }
}

bool LSM6_Read_Temp (int16_t *t)
{
  bool result;
  uint8_t l,h;
  result=bool_read_Byte(STATUS_REG,&l);
  if(l & 0x04)
  {
   result &=bool_read_Byte(OUT_TEMP_L,&l);
   result &=bool_read_Byte(OUT_TEMP_H,&h);
   if(result) *t = (int16_t)( l | h << 8 );
  }
  return result;
}
/*
bool TC74_Read_Temp (int8_t *t)
{
    uint8_t res = 0;

    tx_buf[0] = 0x01;
    tx_buf[1] = 0b00000000;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 0;
    I2C_transfer(i2c, &i2cTransaction);

    while(res == 0)
    {
        tx_buf[0] = 0x01;
        i2cTransaction.slaveAddress = 72;
        i2cTransaction.writeBuf = tx_buf;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rx_buf;
        i2cTransaction.readCount = 1;
        I2C_transfer(i2c, &i2cTransaction);
        res = rx_buf[0] & 0b01000000;
    }
    tx_buf[0] = 0x00;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 1;
    I2C_transfer(i2c, &i2cTransaction);
    res = rx_buf[0] & 0b01000000;
    *t = rx_buf[0];

    tx_buf[0] = 0x01;
    tx_buf[1] = 0b10000000;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 0;
    I2C_transfer(i2c, &i2cTransaction);

    return true;
}
*/
/*float DS1621_Read_Temp (void)
{
    float t;
    int8_t delta;

    tx_buf[0] = 0xEE;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 0;
    I2C_transfer(i2c, &i2cTransaction);

    tx_buf[0] = 0xAA;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 2;
    I2C_transfer(i2c, &i2cTransaction);
    if((int8_t)rx_buf[0] >= 0)
        delta = 1;
    else
        delta = -1;
    t = (int8_t)rx_buf[0] + (rx_buf[1]!=0) * delta * 0.5;

    tx_buf[0] = 0x22;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 0;
    I2C_transfer(i2c, &i2cTransaction);

    return t;
}*/

bool LM75_on ()
{
    tx_buf[0] = 0x01; //normal mode
    tx_buf[1] = 0x00;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 0;
    return I2C_transfer(i2c, &i2cTransaction);
}

bool LM75_off ()
{
    tx_buf[0] = 0x01; //shutdown mode
    tx_buf[1] = 0x01;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 0;
    return I2C_transfer(i2c, &i2cTransaction);

}

bool LM75_Read_Temp (float *t)
{

    bool flag;
    Task_sleep(100000);
    tx_buf[0] = 0x00;
    i2cTransaction.slaveAddress = 72;
    i2cTransaction.writeBuf = tx_buf;

    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rx_buf;
    i2cTransaction.readCount = 2;
    flag=I2C_transfer(i2c, &i2cTransaction);



    *t = ((((int8_t)rx_buf[0] << 8) | rx_buf[1]) >> 5) * 0.125;



    return flag;
}




