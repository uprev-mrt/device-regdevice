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
} mrt_reg_t;

typedef enum{
  MRT_BUS_I2C,
  MRT_BUS_SPI
} mrt_bus_type_e;

#define REG_DEF(name,addr,type,description) \
  mrt_reg_t name = {       \
    .mAddr = addr,              \
    .mSize = sizeof(type),      \
    .mPerm = REG_PERM_ALL       \
  };

typedef struct{
  mrt_bus_type_e mBusType;        //type of device
  mrt_i2c_handle_t  mI2cHandle;   //i2c periph handle
  uint8_t mAddr;                  //i2c address
  mrt_spi_handle_t  mSpiHandle;   //spi handle
  mrt_gpio_t mChipSelect;         //chip select pin
  uint8_t mMemAddrSize;           //size of register address
  int mWriteDelayMS;              //delay after write command
} mrt_regdev_t;


#ifdef __cplusplus

class RegisterDevice {
public:
private:
};

extern "C"
{
#endif

/**
  *@brief creates new generic i2c register device
  *@param handle - i2c periph handle (type defined by platform)
  *@param addr - 8 bit i2c address
  *@param memAddrSize size of register address in bytes
  *@pre Periph should already be configured and initialized
  *@return ptr to new generic register device
  */
mrt_regdev_t* new_i2c_register_device( mrt_i2c_handle_t handle, uint8_t addr, uint8_t memAddrSize );

/**
  *@brief creates new generic spi register device
  *@param handle - spi periph handle (type defined by platform)
  *@param chipSelect - gpio for chip select (type define by platform)
  *@param memAddrSize size of register address in bytes
  *@pre Periph should already be configured and initialized
  *@return ptr to new generic register device
  */
mrt_regdev_t* new_spi_register_device( mrt_spi_handle_t handle, mrt_gpio_t chipSelect, uint8_t memAddrSize );

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

/**
  *@brief writes buffer to address of device
  *@param dev ptr to generic register device
  *@param addr address in memory to write
  *@param data ptr to data to be written
  *param len length of data to write
  *@return status (type defined by platform)
  */
mrt_status_t regdev_write_buf(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len );

/**
  *@brief reads buffer from register address of device
  *@param addr address in memory to read
  *@param reg ptr to register definition
  *@param data ptr to store data
  *param len length of data to read
  *@return bytes read
  */
int regdev_read_buf(mrt_regdev_t* dev,uint32_t addr, uint8_t* data, int len);

/**
  *@brief prints all registers of the devices
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param data ptr to store data
  *@return status (type defined by platform)
  */
mrt_status_t regdev_print_reg(mrt_regdev_t* dev,mrt_reg_t* reg);


//Read and Write functions for each bus type
mrt_status_t regdev_write_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len);
mrt_status_t regdev_read_i2c(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len);

mrt_status_t regdev_write_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len);
mrt_status_t regdev_read_spi(mrt_regdev_t* dev, uint32_t addr, uint8_t* data, int len);



#ifdef __cplusplus
}
}
#endif
