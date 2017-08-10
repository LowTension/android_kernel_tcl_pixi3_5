#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>

extern bool _hwPowerOn(char * powerId, int powerVolt, struct regulator **regVCAM);
extern bool _hwPowerDown(char * powerId, struct regulator **regVCAM);

static int cust_acc_power(struct acc_hw *hw, unsigned int on, char* devname)
{
    if (hw->power_id == MT65XX_POWER_NONE)
        return 0;
    if (on)
        return _hwPowerOn(hw->power_id, hw->power_vol, devname);
    else
        return _hwPowerDown(hw->power_id, devname); 
}

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
    .direction = 7,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
    .power = cust_acc_power,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_kxtj2_1009_cust_acc_hw(void)
{
    return &cust_acc_hw;
}
