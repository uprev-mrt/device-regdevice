/**
  *@file RegisterDevice.c
  *@brief pure c implementation of register device
  *@author Jason Berger
  *@date 03/11/2019
  */



mrt_regdev_t* new_i2c_register_device( mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize )
{
  mrt_regdev_t* newDev = (mrt_reg_t) malloc(sizeof(mrt_reg_t));

  newDev->mBusType = MRT_BUS_I2C;
  newDev->mHandle = handle;
  newDev->mAddr = addr;
  newDev->mMemAddrSize = memAddrSize;
  newDev->mWriteDelayMS = 1;
  newDev->pfWrite = &regdev_write_i2c;   //set write register function to default for i2c
  newDev->pfRead = &regdev_read_i2c;     //set read register function to default for i2c

  return newDev;
}


mrt_regdev_t* new_spi_register_device( mrt_spi_handle_t handle, mrt_gpio_t chipSelect, uint8_t memAddrSize )
{
  newDev->mBusType = MRT_BUS_SPI;
  newDev->mHandle = handle;
  newDev->mChipSelect= chipSelect;
  newDev->mMemAddrSize = memAddrSize;
  newDev->mWriteDelayMS = 1;
  newDev->pfWrite= &regdev_write_spi;   //set write register function to default for spi
  newDev->pfRead = &regdev_read_spi;     //set read register function to default for spi
}


mrt_status_t regdev_write_reg(mrt_regdev_t* dev, mrt_reg_t* reg, uint8_t* data)
{
  return dev->pfWrite(dev, reg->mAddr, data, reg->mSize);
}


int regdev_read_reg(mrt_regdev_t* dev,mrt_reg_t* reg, uint8_t* data)
{
  return dev->pfRead(dev, reg->mAddr, data, reg->mSize);
}

/*      I2C             */
mrt_status_t regdev_write_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
  return MRT_I2C_MEM_WRITE(dev->mHandle, dev->mAddr, addr, dev->mSize , data, len, 5 );
}

mrt_status_t regdev_read_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
  return MRT_I2C_MEM_READ(dev->mHandle, dev->mAddr, addr, dev->mSize , data, len, 5 );
}

/*      SPI               */
mrt_status_t regdev_write_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len)
{
  mrt_status_t status;
  uint8_t trash[len];

  //send address
  status = MRT_SPI_TRANSFER(dev->mHandle, addr, trash, dev->mMemAddrSize, dev->mTimeout );

  //send data
  status = MRT_SPI_TRANSFER(dev->mHandle, data, trash, len, dev->mTimeout)

  return status;
}


mrt_status_t regdev_read_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len)
{
  mrt_status_t status;
  uint8_t trash[len];

  //send address
  status = MRT_SPI_TRANSFER(dev->mHandle, addr, trash, dev->mMemAddrSize, dev->mTimeout );

  //read data
  status = MRT_SPI_TRANSFER(dev->mHandle, trash, data, len, dev->mTimeout)

  return status;
}
}
