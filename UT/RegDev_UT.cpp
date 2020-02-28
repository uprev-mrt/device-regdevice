#ifdef UNIT_TESTING_ENABLED

extern "C"{

#include "../register_device.c"
}


#include <gtest/gtest.h>

mrt_regdev_t regDev;    /* register device */
mrt_reg_t reg1;             /* Dummy Register  */
mrt_reg_t reg2;             /* Dummy Register  */
mrt_reg_t reg3;             /* Dummy Register  */


mrt_status_t dummy_read(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
    switch(addr)
    {
        case 0:
            memcpy((void*)data, (void*)&reg1.mCache, len);
            break;
        case 1:
            memcpy((void*)data, (void*)&reg2.mCache, len);
            break;
        case 2:
            memcpy((void*)data, (void*)&reg3.mCache, len);
            break;
    }

    return MRT_STATUS_OK;
}

mrt_status_t dummy_write(mrt_regdev_t* dev, uint32_t addr, uint8_t* data,int len)
{
    switch(addr)
    {
        case 0:
            memcpy((void*)&reg1.mCache,(void*) data, len);
            break;
        case 1:
            memcpy((void*)&reg2.mCache, (void*)data, len);
            break;
        case 2:
            memcpy((void*)&reg3.mCache, (void*)data, len);
            break;
    }

    return MRT_STATUS_OK;
} 


TEST(RegDevice, init )
{
  init_i2c_register_device(&regDev, 0, 0, 1 );
  REG_INIT_EXP( reg1 , 0 , uint8_t, REG_PERM_RW , 0x45  );
  REG_INIT_EXP( reg2 , 1 , uint16_t, REG_PERM_RW , 0x6767 );
  REG_INIT_EXP( reg3 , 2 , uint32_t, REG_PERM_RW , 0x8989898989  );

  regDev.fRead = dummy_read;
  regDev.fWrite = dummy_write;


  ASSERT_EQ(reg1.mCache,0x45);
  ASSERT_EQ(reg1.mAddr, 0);
}


TEST(RegDevice, write )
{
    /* write and verify reg1 */
    regdev_write_reg(&regDev, &reg1, 0xAB);
    uint8_t tp1 = regdev_read_reg(&regDev, &reg1);
    ASSERT_EQ(tp1, 0xAB);

    /* write and verify reg2 */
    regdev_write_reg(&regDev, &reg2, 0xCDEF);
    uint16_t tp2 = regdev_read_reg(&regDev, &reg2);
    ASSERT_EQ(tp2, 0xCDEF);

    /* write and verify reg3 */
    regdev_write_reg(&regDev, &reg3, 0xDEADBEEF);
    uint32_t tp3 = regdev_read_reg(&regDev, &reg3);
    ASSERT_EQ(tp3, 0xDEADBEEF);

}




#endif
