/**
  *@file RegisterDevice.h
  *@brief definition of abstract register based RegDevice
  *@author Jason Berger
  *@date 02/16/2019
  */

#ifdef __cplusplus
namespace Devices
{
#ednif

#define REG_PERM_R    0x01
#define REG_PERM_W    0x02
#define REG_PERM_ALL  0xFF

typedef struct{
  uint32_t mAddr;
  int mSize;
  uint8_t mPerm;
  bool mTouch;
} register_def_t;

#define REG_DEF(name,addr,type,description) \
  register_def_t name = {       \
    .mAddr = addr,              \
    .mSize = sizeof(type),      \
    .mPerm = REG_PERM_ALL       \
  };

typedef struct{
  mrt_i2c_handle_t  mI2cHandle;
  uint8_t mAddr;
  uint8_t mMemAddrSize;
  int mWriteDelay;
} regdev_t;


#ifdef __cplusplus

class RegisterDevice {
public:
  RegisterDevice();
  RegisterDevice(mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize );
  virtual void writeRegister(register_def_t* reg, uint8_t* data);
  virtual void readRegister(register_def_t* reg, uint8_t* data);
  virtual void printRegister(register_def_t* reg);
private:
};

extern "C"
{
#endif

regdev_t* new_register_device(regdev_t* pRegDev, mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize );
void regdev_write_register(regdev_t* pRegDev, register_def_t* reg, uint8_t* data);
void regdev_read_register(regdev_t* pRegDev,register_def_t* reg, uint8_t* data);
void regdev_print_register(regdev_t* pRegDev,register_def_t* reg);

#ifdef __cplusplus
}
}
#endif
