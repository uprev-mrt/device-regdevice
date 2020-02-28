/**
  *@file RegisterDevice.h
  *@brief definition of abstract register based RegDevice
  *@author Jason Berger
  *@date 02/16/2019
  */

 #include "Platforms/Common/mrt_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* REGDEV Debug macro*/
#ifdef MRT_REGDEV_DEBUG_ENABLE
  #define MRT_REGDEV_DEBUG(f_, ...)  \
  MRT_PRINTF("[RegDev] ");       \
  MRT_PRINTF((f_), ##__VA_ARGS__ );     \
  MRT_PRINTF("\n")
#else 
  #define MRT_REGDEV_DEBUG(f_, ...)
#endif

#define REG_PERM_R    0x01
#define REG_PERM_W    0x02
#define REG_PERM_RW   0x03
#define REG_FLAG_SWAP 0x80 //Flag to swap endianess

#define REG_ACCESS_NONE 0x00
#define REG_ACCESS_R    0x01
#define REG_ACCESS_W    0x02


typedef struct{
  uint32_t mAddr;
  uint8_t mSize;
  struct {
    uint8_t mPerm: 2;   //Permissions for register
    uint8_t mLastAccess: 2;   //Last access type
    uint8_t mHistory: 2;   //Flags to indicate if register was ever read/written
  }mFlags;
#ifndef MRT_REGDEV_DISABLE_CACHE
  uint32_t mCache;
#endif
} mrt_reg_t;

#define REG_DEF(name,addr,type, perm, default) \
  mrt_reg_t name = {           \
    .mAddr = addr,             \
    .mSize = sizeof(type),     \
    .mFlags = perm,      \
    .mCache = default           \
  };                            

#define REG_INIT(reg,addr,type, perm, default) \
(reg) = (mrt_reg_t){          \
  .mAddr = addr,              \
  .mSize = sizeof(type),      \
  .mFlags.mPerm = perm,        \
  .mCache = default            \
}          

/* Used in unit testing because C++ does not support compound literals.. */
#define REG_INIT_EXP(reg,addr,type, perm, default) \
  (reg).mAddr = addr; \
  (reg).mSize = sizeof(type); \
  (reg).mFlags.mPerm = perm; \
  (reg).mCache = default;

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
  bool mAutoIncrement;            
  uint32_t mAiMask;
  mrt_endianess_e mEndianess;     //indicates endianess of device to automate byte-swapping
  RegOperation fWrite;            //pointer to write function
  RegOperation fRead;             //pointer to read function
};



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
  *@param data data to be written
  *@return status (type defined by platform)
  */
mrt_status_t regdev_write_reg(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t data);

/**
  *@brief reads register of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@return register data
  */
uint32_t regdev_read_reg(mrt_regdev_t* dev,mrt_reg_t* reg);

/**
  *@brief writes register of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param mask mask of field to write
  *@param data data to be written
  *@return status (type defined by platform)
  */
mrt_status_t regdev_write_field(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask, uint32_t data);

/**
  *@brief reads register of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param mask mask of field to read
  *@return register data
  */
uint32_t regdev_read_field(mrt_regdev_t* dev,mrt_reg_t* reg, uint32_t mask);

/**
  *@brief Sets flags of register
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param mask mask of flags to be set
  *@return status (type defined by platform)
  */
mrt_status_t regdev_set_flags(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask);

/**
  *@brief writes register of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@param mask mask of flags to be cleared
  *@return status (type defined by platform)
  */
mrt_status_t regdev_clear_flags(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask);

/**
  *@brief checks flags of device
  *@param dev ptr to generic register device
  *@param reg ptr to register definition
  *@return true if all flags in mask are set 
  *@return false if any flags in mask are not set
  */
bool regdev_check_flags(mrt_regdev_t* dev, mrt_reg_t* reg, uint32_t mask);



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
#endif
