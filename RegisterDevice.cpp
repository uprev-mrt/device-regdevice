/**
  *@file RegisterDevice.cpp
  *@brief Base class for register based devices
  *@author Jason Berger
  *@date 02/17/2019
  */

namespace Devices
{

RegisterDevice::RegisterDevice()
{
  mAddr = 0;
  mWriteDelay = 2;
  mMemAddrSize = 2;
}

RegisterDevice::RegisterDevice(mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize )
{
  mAddr = addr;
  mHandle = handle;
  mWriteDelay = 2;
  mMemAddrSize = memAddrSize;
}

void RegisterDevice::writeRegister(register_def_t* reg, uint8_t* data)
{
  MRT_I2C_MEM_WRITE(mHandle, mAddr, reg->mAddr, mMemAddrSize, data, reg->mSize, 500 );
  MRT_DELAY_MS(mWriteDelay);
}

void RegisterDevice::readRegister(register_def_t* reg, uint8_t* data)
{
  MRT_I2C_MEM_READ(mHandle, mAddr, reg->mAddr, mMemAddrSize, data, reg->mSize, 500 );
  MRT_DELAY_MS(mWriteDelay);
}

void RegisterDevice::printRegister(register_def_t* reg)
{

}

extern "C"
{

regdev_t* new_register_device(regdev_t* pRegDev, mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize )
{
  RegisterDevice* p = new RegisterDevice(handle,addr,memAddrSize);
  return (RegisterDevice*)p;
}

void regdev_write_register(regdev_t* pRegDev, register_def_t* reg, uint8_t* data)
{
  static_cast<RegisterDevice*>(pRegDev)->writeRegister(reg,data);
}

void regdev_read_register(regdev_t* pRegDev,register_def_t* reg, uint8_t* data)
{
  static_cast<RegisterDevice*>(pRegDev)->readRegister(reg,data);
}

void regdev_print_register(regdev_t* pRegDev,register_def_t* reg)
{
  static_cast<RegisterDevice*>(pRegDev)->printRegister(reg);
}

}
}
