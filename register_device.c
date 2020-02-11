/**
  *@file RegisterDevice.c
  *@brief pure c implementation of register device
  *@author Jason Berger
  *@date 03/11/2019
  */

#include "register_device.h"

/**
 * @brief Get the offset of a mask
 * @param mask 
 * @return int 
 */
static int getMaskOffset(uint32_t mask)
{
  int count =0;
  while((mask & 1) == 0)
  {
    mask = mask >> 1;
    count++;
  }

  return count;
}


mrt_status_t init_i2c_register_device(mrt_regdev_t* dev, mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize )
{

  dev->mBusType = MRT_BUS_I2C;
  dev->mI2cHandle = handle;
  dev->mAddr = addr;
  dev->mMemAddrSize = memAddrSize;
  dev->mWriteDelayMS = 1;
  dev->fWrite = regdev_write_i2c;   //set write register function to default for i2c
  dev->fRead = regdev_read_i2c;     //set read register function to default for i2c
  dev->mAutoIncrement = false;
  dev->mAiMask =0;
  dev->mTimeout = 5;

  return MRT_STATUS_OK;
}


mrt_status_t init_spi_register_device(mrt_regdev_t* dev, mrt_spi_handle_t handle, mrt_gpio_t chipSelect, uint8_t memAddrSize )
{
  dev->mBusType = MRT_BUS_SPI;
  dev->mSpiHandle = handle;
  dev->mChipSelect= chipSelect;
  dev->mMemAddrSize = memAddrSize;
  dev->mWriteDelayMS = 1;
  dev->fWrite= regdev_write_spi;   //set write register function to default for spi
  dev->fRead = regdev_read_spi;     //set read register function to default for spi
  dev->mAutoIncrement = false;
  dev->mAiMask =0;
  dev->mTimeout = 5;

  return MRT_STATUS_OK;
}


mrt_status_t regdev_write_reg(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t data)
{
  mrt_status_t ret;

    /* If register does not have write permission, return 0*/
  if(!(reg->mFlags.mPerm & REG_ACCESS_W))
    return MRT_STATUS_ERROR;


#ifndef MRT_REGDEV_DISABLE_CACHE
  reg->mCache = data;
#endif


  MRT_REGDEV_DEBUG("Writing 0x%04X to register 0x%04X", data, reg->mAddr);
  reg->mFlags.mLastAccess = REG_ACCESS_W;
  reg->mFlags.mHistory |= REG_ACCESS_W;

  if(dev->mAutoIncrement)
    ret = dev->fWrite(dev, (reg->mAddr | dev->mAiMask), &data, reg->mSize);
  else 
    ret = dev->fWrite(dev, reg->mAddr, &data, reg->mSize);
  MRT_DELAY_MS(dev->mWriteDelayMS);
  return ret;
}


uint32_t regdev_read_reg(mrt_regdev_t* dev,mrt_reg_t* reg)
{
  uint32_t data = 0;

  /* If register does not have read permission, return cache since it will have the latest written value */
  if(!(reg->mFlags.mPerm & REG_ACCESS_R))
    return reg->mCache;

  if(dev->mAutoIncrement)
    dev->fRead(dev, (reg->mAddr | dev->mAiMask), &data, reg->mSize);
  else 
    dev->fRead(dev, reg->mAddr, &data, reg->mSize);
  
  
  MRT_REGDEV_DEBUG("Read 0x%04X from register 0x%04X", data, reg->mAddr);
  reg->mFlags.mLastAccess = REG_ACCESS_R;
  reg->mFlags.mHistory |= REG_ACCESS_R;
  
#ifndef MRT_REGDEV_DISABLE_CACHE
  reg->mCache = data;
#endif
  return data;
}

mrt_status_t regdev_write_field(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask, uint32_t data )
{
  mrt_status_t status;
  int offset = getMaskOffset(mask);         //get offset of field
  uint32_t val = regdev_read_reg(dev,reg);  //get current value
  
  val &= (~mask);                           //clear the bits for the field
  val |= ((data << offset) & mask);         //set bits from data 

  status = regdev_write_reg(dev,reg,val);

  return status;
}


uint32_t regdev_read_field(mrt_regdev_t* dev,mrt_reg_t* reg, uint32_t mask)
{
  int offset = getMaskOffset(mask);        //get offset of field
  uint32_t val = regdev_read_reg(dev,reg); //get current value
  
  val = (val & mask) >> offset;            //mask off field and remove offset

  return val;
}

mrt_status_t regdev_set_flags(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask)
{
  mrt_status_t status;

  uint32_t val = regdev_read_reg(dev, reg); //read current value 
  val |= mask;                              //set flags 

  status = regdev_write_reg(dev,reg, val); // write new value
  return status;
}


mrt_status_t regdev_clear_flags(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask)
{
    mrt_status_t status;

    uint32_t val = regdev_read_reg(dev, reg); //read current value 
    val &= (~mask);                              //clear flags 

    status = regdev_write_reg(dev,reg, val); // write new value
    return status;
}


bool regdev_check_flags(mrt_regdev_t* dev,mrt_reg_t* reg, uint32_t mask)
{
  uint32_t val = regdev_read_reg(dev, reg); //read current value 
  val &= mask;                              //set flags 

  if(val == mask)
  {
    return true;
  }

  return false;
}

/*      I2C             */
mrt_status_t regdev_write_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
  return MRT_I2C_MEM_WRITE(dev->mI2cHandle, dev->mAddr, addr, dev->mMemAddrSize , data, len, dev->mTimeout );
}

mrt_status_t regdev_read_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
  return MRT_I2C_MEM_READ(dev->mI2cHandle, dev->mAddr, addr, dev->mMemAddrSize , data, len, dev->mTimeout );
}

/*      SPI               */
mrt_status_t regdev_write_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len)
{
  mrt_status_t status;
  uint8_t trash[len];

  //send address
  status = MRT_SPI_TRANSFER(dev->mSpiHandle, addr, trash, dev->mMemAddrSize, dev->mTimeout );

  //send data
  status = MRT_SPI_TRANSFER(dev->mSpiHandle, data, trash, len, dev->mTimeout);

  return status;
}


mrt_status_t regdev_read_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len)
{
  mrt_status_t status;
  uint8_t trash[len];

  //send address
  status = MRT_SPI_TRANSFER(dev->mSpiHandle, addr, trash, dev->mMemAddrSize, dev->mTimeout );

  //read data
  status = MRT_SPI_TRANSFER(dev->mSpiHandle, trash, data, len, dev->mTimeout);

  return status;
}
