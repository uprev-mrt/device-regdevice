/**
  *@file RegisterDevice.h
  *@brief definition of abstract register based RegDevice
  *@author Jason Berger
  *@date 02/16/2019
  */

 #include "Platforms/Common/mrt_platform.h"

#ifdef __cplusplus
namespace Devices
{
#endif

#define REG_PERM_R    0x01
#define REG_PERM_W    0x02
#define REG_PERM_RW   0x03
#define REG_FLAG_SWAP 0x80 //Flag to swap endianess

typedef struct{
  uint32_t mAddr;
  int mSize;
  uint8_t mFlags;
  bool mTouch;
} mrt_reg_t;

#define REG_DEF(name,addr,type, flags,description) \
  mrt_reg_t name##_data = {     \
    .mAddr = addr,              \
    .mSize = sizeof(type),      \
    .mFlags = flags      \
  };                            \
  mrt_reg_t* name = &name##_data;


typedef struct mrt_regdev_t mrt_regdev_t;
typedef mrt_status_t (*RegOperation)(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len); 

struct mrt_regdev_t{
  mrt_bus_type_e mBusType;        //type of device
  mrt_i2c_handle_t  mI2cHandle;   //i2c periph handle
  uint8_t mAddr;                  //i2c address
  mrt_spi_handle_t  mSpiHandle;   //spi handle
  mrt_gpio_t mChipSelect;         //chip select pin
  uint8_t mMemAddrSize;           //size of register address
  int mWriteDelayMS;              //delay after write command
  int mTimeout;
  mrt_endianess_e mEndianess;
  RegOperation fWrite; //pointer to write function
  RegOperation fRead;  //pointer to read function
};


#ifdef __cplusplus
extern "C"
{
#endif

/**
  *@brief creates new generic i2c register device
  *@param dev ptr to regdev
  *@param handle - i2c periph handle (type defined by platform)
  *@param addr - 8 bit i2c address
  *@param memAddrSize size of register address in bytes
  *@pre Periph should already be configured and initialized
  *@return status
  */
mrt_status_t init_i2c_register_device(mrt_regdev_t* dev, mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize );

/**
  *@brief creates new generic spi register device
  *@param dev ptr to regdev
  *@param handle - spi periph handle (type defined by platform)
  *@param chipSelect - gpio for chip select (type define by platform)
  *@param memAddrSize size of register address in bytes
  *@pre Periph should already be configured and initialized
  *@return status
  */
mrt_status_t init_spi_register_device(mrt_regdev_t* dev, mrt_spi_handle_t handle, mrt_gpio_t chipSelect, uint8_t memAddrSize );

/**
  *@brief writes register of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param data ptr to data to be written
  *@return status (type defined by platform)
  */
mrt_status_t regdev_write_reg(mrt_regdev_t* dev, mrt_reg_t* reg, uint8_t* data);

/**
  *@brief reads register of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param data ptr to store data
  *@return bytes read
  */
int regdev_read_reg(mrt_regdev_t* dev,mrt_reg_t* reg, uint8_t* data);


/***************  DEFAULTS        *****************
 * these are the defaults pointed to by mrt_regdev_t->pfWrite() and mrt_regdev_t->pfRead()
 *    Depending on bus type. to override them , write your own function and point the member function pointers to it;
 */

/**
  *@brief writes buffer to address of device
  *@param dev ptr to generic register device
  *@param addr address in memory to write
  *@param data ptr to data to be written
  *param len length of data to write
  *@return status (type defined by platform)
  */
mrt_status_t regdev_write_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len );
mrt_status_t regdev_write_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len);

/**
  *@brief reads buffer from register address of device
  *@param addr address in memory to read
  *@param reg ptr to register definition
  *@param data ptr to store data
  *param len length of data to read
  *@return status (type defined by platform)
  */
mrt_status_t regdev_read_i2c(mrt_regdev_t* dev,uint32_t addr, uint8_t* data, int len);
mrt_status_t regdev_read_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len);


#ifdef __cplusplus
}
}
#endif
