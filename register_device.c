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

  return newDev;
}


mrt_regdev_t* new_spi_register_device( mrt_spi_handle_t handle, mrt_gpio_t chipSelect, uint8_t memAddrSize )
{
  newDev->mBusType = MRT_BUS_SPI;
  newDev->mHandle = handle;
  newDev->mChipSelect= chipSelect;
  newDev->mMemAddrSize = memAddrSize;
  newDev->mWriteDelayMS = 1;
}


mrt_status_t regdev_write_reg(mrt_regdev_t* dev, mrt_reg_t* reg, uint8_t* data)
{
  return regdev_write_buf(dev, reg->mAddr, data, reg->mSize);
}


int regdev_read_reg(mrt_regdev_t* dev,mrt_reg_t* reg, uint8_t* data)
{
  return regdev_read_buf(dev, reg->mAddr, data, reg->mSize);
}


mrt_status_t regdev_write_buf(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len )
{
  mrt_status_t status;
  switch (dev->mBusType)
  {
    case MRT_BUS_I2C:
      status = regdev_write_i2c(dev, addr, data, len);
      break;
    case MRT_BUS_SPI:
      status = regdev_write_spi(dev, addr, data, len);
      break;
  }

  return status;
}


int regdev_read_buf(mrt_regdev_t* dev,uint32_t addr, uint8_t* data, int len)
{
  int byteCount;
  switch (dev->mBusType)
  {
    case MRT_BUS_I2C:
      byteCount = regdev_write_i2c(dev, addr, data, len);
      break;
    case MRT_BUS_SPI:
      byteCount = regdev_write_spi(dev, addr, data, len);
      break;
  }

  return byteCount;
}


mrt_status_t regdev_print_reg(mrt_regdev_t* dev,mrt_reg_t* reg);


mrt_status_t regdev_write_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len);
mrt_status_t regdev_read_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len);

mrt_status_t regdev_write_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len);
mrt_status_t regdev_read_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len);