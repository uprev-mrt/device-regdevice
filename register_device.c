/**
  *@file RegisterDevice.c
  *@brief pure c implementation of register device
  *@author Jason Berger
  *@date 03/11/2019
  */

#include "register_device.h"


mrt_status_t init_i2c_register_device(mrt_regdev_t* dev, mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize )
{

  dev->mBusType = MRT_BUS_I2C;
  dev->mI2cHandle = handle;
  dev->mAddr = addr;
  dev->mMemAddrSize = memAddrSize;
  dev->mWriteDelayMS = 1;
  dev->fWrite = regdev_write_i2c;   //set write register function to default for i2c
  dev->fRead = regdev_read_i2c;     //set read register function to default for i2c

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

  return MRT_STATUS_OK;
}


mrt_status_t regdev_write_reg(mrt_regdev_t* dev, mrt_reg_t* reg, uint8_t* data)
{
  mrt_status_t ret;
  ret = dev->fWrite(dev, reg->mAddr, data, reg->mSize);
  MRT_DELAY_MS(dev->mWriteDelayMS);
  return ret;
}


int regdev_read_reg(mrt_regdev_t* dev,mrt_reg_t* reg, uint8_t* data)
{
  return dev->fRead(dev, reg->mAddr, data, reg->mSize);
}

/*      I2C             */
mrt_status_t regdev_write_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
  return MRT_I2C_MEM_WRITE(dev->mI2cHandle, dev->mAddr, addr, dev->mMemAddrSize , data, len, 5 );
}

mrt_status_t regdev_read_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
  return MRT_I2C_MEM_READ(dev->mI2cHandle, dev->mAddr, addr, dev->mMemAddrSize , data, len, 5 );
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
