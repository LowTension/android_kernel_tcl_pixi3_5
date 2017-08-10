/* Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("HUAQIN SOFTWARE")
 * RECEIVED FROM HUAQIN AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. HUAQIN EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES HUAQIN PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE HUAQIN SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN HUAQIN SOFTWARE. HUAQIN SHALL ALSO NOT BE RESPONSIBLE FOR ANY HUAQIN
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND HUAQIN'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE HUAQIN SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT HUAQIN'S OPTION, TO REVISE OR REPLACE THE HUAQIN SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * HUAQIN FOR SUCH HUAQIN SOFTWARE AT ISSUE.
 *

 */


/*******************************************************************************
* Dependency
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/mtd/mtd.h>
//#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#include <mach/mt_boot.h>


#include <linux/proc_fs.h>


#include "lcm_drv.h"
#include "../../../drivers/input/touchscreen/mediatek/tpd.h"


#define HARDWARE_INFO_VERSION   0x7502


extern LCM_DRIVER  *g_lcm_drv; 
//extern char *lcm_temp_name; //temp lcm name 

//extern flashdev_info devinfo;
extern char *g_main_camera;
extern char *g_sub_camera;
/****************************************************************************** 
 * Function Configuration
******************************************************************************/


/****************************************************************************** 
 * Debug configuration
******************************************************************************/


//hardware info driver
static ssize_t show_hardware_info(struct device *dev,struct device_attribute *attr, char *buf)
{   
    #if 0
    int ret_value = 1;
    int count = 0;

    //show lcm driver 
    printk("show_lcm_info return lcm name\n");
    count += sprintf(buf, "LCM Name:   ");//12 bytes
    if(lcm_drv->name)   
        count += sprintf(buf+count, "%s\n", lcm_drv->name); 
    else
        count += sprintf(buf+count,"no lcd name found\n");


    //show nand drvier name
    printk("show nand device name\n");
    count += sprintf(buf+count, "NAND Name:  ");//12 bytes
    if(devinfo.devciename) 
        count += sprintf(buf+count, "%s\n", devinfo.devciename);
    else
        count += sprintf(buf+count,"no nand name found\n");    


    //show Main Camera drvier name
    printk("show Main Camera device name\n");
    count += sprintf(buf+count, "Main Camera:");//12 bytes
    if(g_main_camera) 
        count += sprintf(buf+count, "%s\n", g_main_camera);
    else
        count += sprintf(buf+count,"pls turn on camera once\n");

    //show Sub Camera drvier name
    printk("show Sub camera name\n");
    count += sprintf(buf+count, "Sub Camera: ");//12 bytes
    if(g_sub_camera) 
        count += sprintf(buf+count, "%s\n", g_sub_camera);
    else
        count += sprintf(buf+count,"pls turn on camera once\n");

    
    //A20 using 6620
    //show WIFI
    printk("show WIFI name\n");
    count += sprintf(buf+count, "WIFI:       ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no WIFI name found\n");

    printk("show Bluetooth name\n");
    count += sprintf(buf+count, "Bluetooth:  ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no Bluetooth name found\n");

    printk("show GPS name\n");
    count += sprintf(buf+count, "GPS:        ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no GPS name found\n");

    printk("show FM name\n");
    count += sprintf(buf+count, "FM:         ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no FM name found\n");

    
    ret_value = count;
    #endif

    return 0;
}
static ssize_t store_hardware_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


static ssize_t show_version(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    
    ret_value = sprintf(buf, "Version:    :0x%x\n", HARDWARE_INFO_VERSION);     

    return ret_value;
}
static ssize_t store_version(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}




static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    if(g_lcm_drv != NULL)
    	ret_value = sprintf(buf, "lcd name    :%s\n", g_lcm_drv->name);  
	else
    	ret_value = sprintf(buf, "lcd name    :%s\n", "not_found");  

    return ret_value;
}
static ssize_t store_lcm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


extern struct tpd_device  *tpd;
extern struct tpd_driver_t *g_tpd_drv;



static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    count += sprintf(buf + count, "ctp driver  :%s\n", g_tpd_drv->tpd_device_name);

    //count += sprintf(buf + count, "ctp version :0x%x\n", tpd->dev->id.version); 
    count += sprintf(buf + count, "ctp version :%x\n", tpd->hq_ctp_firmware_version); 

    if(tpd->hq_ctp_module_id)
        count += sprintf(buf + count, "ctp modlue id  :%d\n", tpd->hq_ctp_module_id);
    if(tpd->hq_ctp_module_name)
        count += sprintf(buf + count, "ctp modlue  :%s\n", tpd->hq_ctp_module_name);
    
    ret_value = count;
    
    return ret_value;
}
static ssize_t store_ctp(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

   
    if(g_main_camera)
        ret_value = sprintf(buf , "main camera :%s\n", g_main_camera);
    else
        ret_value = sprintf(buf , "main camera :PLS TURN ON CAMERA FIRST\n");
    
    
    return ret_value;
}
static ssize_t store__main_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    if(g_sub_camera)
        ret_value = sprintf(buf , "sub camera  :%s\n", g_sub_camera);
    else
        ret_value = sprintf(buf , "sub camera  :PLS TURN ON CAMERA FIRST\n");

    
    return ret_value;
}
static ssize_t store_sub_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}



//MT6620 names
static ssize_t show_wifi(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "wifi name   :MT6627\n");     
    return ret_value;
}
static ssize_t show_bt(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "bt name     :MT6627\n");     
    return ret_value;
}
static ssize_t show_gps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "GPS name    :MT6627\n");     
    return ret_value;
}
static ssize_t show_fm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "FM name     :MT6627\n");     
    return ret_value;
}




static ssize_t store_wifi(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
static ssize_t store_bt(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
static ssize_t store_gps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
static ssize_t store_fm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

char *g_alsps_name  = NULL;
static ssize_t show_alsps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_ALSPS)
    if(g_alsps_name)
        ret_value = sprintf(buf, "AlSPS name  :%s\n",g_alsps_name);     
    else
        ret_value = sprintf(buf, "AlSPS name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "AlSPS name  :Not support ALSPS\n");   
    #endif
    return ret_value;
}

static ssize_t store_alsps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


char *g_gsensor_name  = NULL;
static ssize_t show_gsensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
    if(g_gsensor_name)
        ret_value = sprintf(buf, "GSensor name:%s\n",g_gsensor_name);   
    else
        ret_value = sprintf(buf, "GSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "GSensor name:Not support GSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_gsensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

char *g_msensor_name  = NULL;
static ssize_t show_msensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
    if(g_msensor_name)
        ret_value = sprintf(buf, "MSensor name:%s\n",g_msensor_name);   
    else
        ret_value = sprintf(buf, "MSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "MSensor name:Not support MSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_msensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

char *g_Gyro_name  = NULL;
static ssize_t show_gyro(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
    if(g_Gyro_name)
        ret_value = sprintf(buf, "Gyro  name  :%s\n",g_Gyro_name);  
    else
        ret_value = sprintf(buf, "Gyro  name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "Gyro  name  :Not support Gyro\n");    
    #endif
    return ret_value;
}

static ssize_t store_gyro(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

//kaka_14_1113 add
extern unsigned int msdc_get_capacity(int get_emmc_total);

unsigned int round_kbytes_to_readable_mbytes(unsigned int k){
       unsigned int r_size_m = 0;

       if(k > 64*1024*1024){
               r_size_m = 128*1024;
       }else if(k > 32*1024*1024){
               r_size_m = 64*1024;
       }else if(k > 16*1024*1024){
               r_size_m = 32*1024;
       }else if(k > 8*1024*1024){
               r_size_m = 16*1024;
       }else if(k > 4*1024*1024){
               r_size_m = 8*1024;
       }else if(k > 2*1024*1024){
               r_size_m = 4*1024;
       }else if(k > 1*1024*1024){
               r_size_m = 2*1024;
       }else if(k > 512*1024){
               r_size_m = 1024;
       }else if(k > 256*1024){
               r_size_m = 512;
       }else if(k > 128*1024){
               r_size_m = 256;
       }else{
               k = 0;
       }
       
       return r_size_m;
}
	 
static ssize_t show_ramsize(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
		 
    #define K(x) ((x) << (PAGE_SHIFT - 10))
    struct sysinfo i;
    si_meminfo(&i);
	 
    if(round_kbytes_to_readable_mbytes(K(i.totalram)) >= 1024){
       ret_value = sprintf(buf,"%dGB",round_kbytes_to_readable_mbytes(K(i.totalram))/1024);
    }else{
       ret_value = sprintf(buf,"%dMB",round_kbytes_to_readable_mbytes(K(i.totalram)));
    }
		 
		 
    return ret_value;
}
		 
static ssize_t show_flashsize(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
   
    struct sysinfo i;
    si_meminfo(&i);
    
    ret_value = sprintf(buf,"%dGB",round_kbytes_to_readable_mbytes(msdc_get_capacity(1)/2)/1024);
    return ret_value;
}

//kaak_14_1113 end






static DEVICE_ATTR(99_version, 0644, show_version, store_version);
static DEVICE_ATTR(00_lcm, 0644, show_lcm, store_lcm);
static DEVICE_ATTR(01_ctp, 0644, show_ctp, store_ctp);
	//begin: modify by fangjie for MMITest show camera info
//old: static DEVICE_ATTR(02_main_camera, 0644, show_main_camera, store__main_camera);
//old: static DEVICE_ATTR(03_sub_camera, 0644, show_sub_camera, store_sub_camera);
static DEVICE_ATTR(main_camera, 0644, show_main_camera, store__main_camera);
static DEVICE_ATTR(sub_camera, 0644, show_sub_camera, store_sub_camera);
	//begin: modify by fangjie for MMITest show camera info

//static DEVICE_ATTR(04_flash, 0666, show_flash, store_flash);
static DEVICE_ATTR(05_gsensor, 0644, show_gsensor, store_gsensor);
static DEVICE_ATTR(06_msensor, 0644, show_msensor, store_msensor);
static DEVICE_ATTR(08_gyro, 0644, show_gyro, store_gyro);


static DEVICE_ATTR(07_alsps, 0644, show_alsps, store_alsps);


static DEVICE_ATTR(09_wifi, 0644, show_wifi, store_wifi);
static DEVICE_ATTR(10_bt, 0644, show_bt, store_bt);
static DEVICE_ATTR(11_gps, 0644, show_gps, store_gps);
static DEVICE_ATTR(12_fm, 0644, show_fm, store_fm);

//kaka_14_1113 add
static DEVICE_ATTR(20_ramsize,0644,show_ramsize,NULL);
static DEVICE_ATTR(21_flashsize,0644,show_flashsize,NULL);
//kaka_14_1113 end

///////////////////////////////////////////////////////////////////////////////////////////
//// hq factory apk support
///////////////////////////////////////////////////////////////////////////////////////////

#define PROC_NAME1	"boot_status"
static struct proc_dir_entry *fts_proc_entry1;
extern BOOTMODE get_boot_mode_ex(void);//Huangyisong_add 20130823 start
static int hq_apk_debug_read1( struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int i, len, err = -1;
	char buf[1];

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);	
	if (!page) 
	{				
		kfree(page);			
		return -ENOMEM; 
	}
	printk("get_boot_mode_ex =%d\n", get_boot_mode_ex());
	ptr = page; 
	if (11 == get_boot_mode_ex())
	{
		buf[0] = 1;
	}
	else if(12 == get_boot_mode_ex())
	{
		buf[0] = 2;
	}
	else if(13 == get_boot_mode_ex())// //demomode
	{
		buf[0] = 3;
	}
	else
	{
		buf[0] = 0;
	}

	ptr += sprintf(ptr, "%c", buf[0]);


	len = ptr - page;								
	if(*ppos >= len)
	{				
		kfree(page);			
		return 0;		
	}		
	err = copy_to_user(buffer,(char *)page,len);					
	*ppos += len;	
	if(err) 
	{				
		kfree(page);			
		return err; 	
	}		
	kfree(page);	
	return len; 	
}

static const struct file_operations fts_proc_fops = { 
    .write = NULL,
    .read = hq_apk_debug_read1,
};


int hq_apk_create_proc_boot_status(void )
{
	fts_proc_entry1 = proc_create(PROC_NAME1, 0777, NULL,&fts_proc_fops);
	if (NULL == fts_proc_entry1) {
		printk("hq_hardwareinfo proc_creat faild");
		return -ENOMEM;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int HardwareInfo_driver_probe(struct platform_device *dev)   
{   
    int ret_device_file = 0;

    printk("** HardwareInfo_driver_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_00_lcm)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_01_ctp)) != 0) goto exit_error;
	//begin: modify by fangjie for MMITest show camera info
   //old:  if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_02_main_camera)) != 0) goto exit_error;
   //old:  if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_03_ub_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_sub_camera)) != 0) goto exit_error;
	//end: modify by fangjie for MMITest show camera info  
    //if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_04_flash)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_05_gsensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_06_msensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_08_gyro)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_07_alsps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_09_wifi)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_10_bt)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_11_gps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_12_fm)) != 0) goto exit_error;
   

   	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_99_version)) != 0) goto exit_error;     // create hq apk ftm proc file
    //kaka_14_1113 add
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_20_ramsize)) != 0) goto exit_error; 
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_21_flashsize)) != 0) goto exit_error; 
    //kaka_14_1113 end
	hq_apk_create_proc_boot_status();

exit_error: 
    return ret_device_file;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
    printk("** HardwareInfo_drvier_remove!! **");

    device_remove_file(&(dev->dev), &dev_attr_00_lcm);
    device_remove_file(&(dev->dev), &dev_attr_01_ctp);
    //begin: modify by fangjie for MMITest show camera info
    //old:device_remove_file(&(dev->dev), &dev_attr_02_main_camera); 
   //old: device_remove_file(&(dev->dev), &dev_attr_03_sub_camera);
    device_remove_file(&(dev->dev), &dev_attr_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_sub_camera);
   //end: modify by fangjie for MMITest show camera info
    
    //device_remove_file(&(dev->dev), &dev_attr_04_flash);
    device_remove_file(&(dev->dev), &dev_attr_05_gsensor);
    device_remove_file(&(dev->dev), &dev_attr_06_msensor);
    device_remove_file(&(dev->dev), &dev_attr_08_gyro);
    device_remove_file(&(dev->dev), &dev_attr_07_alsps);
    device_remove_file(&(dev->dev), &dev_attr_09_wifi);
    device_remove_file(&(dev->dev), &dev_attr_10_bt);
    device_remove_file(&(dev->dev), &dev_attr_11_gps);
    device_remove_file(&(dev->dev), &dev_attr_12_fm);
    
    //kaka_14_1113 add
    device_remove_file(&(dev->dev), &dev_attr_20_ramsize);
    device_remove_file(&(dev->dev), &dev_attr_21_flashsize);
    //kaka_14_1113 end
    
	device_remove_file(&(dev->dev), &dev_attr_99_version);
    return 0;
}





static struct platform_driver HardwareInfo_driver = {
    .probe      = HardwareInfo_driver_probe,
    .remove     = HardwareInfo_driver_remove,
    .driver     = {
        .name = "HardwareInfo",
    },
};

static struct platform_device HardwareInfo_device = {
    .name   = "HardwareInfo",
    .id     = -1,
};




static int __init HardwareInfo_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&HardwareInfo_device);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }
    

    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

    goto ok_result;

    
fail_1:
    platform_driver_unregister(&HardwareInfo_driver);
fail_2:
    platform_device_unregister(&HardwareInfo_device);
ok_result:

    return ret;
}


/*****************************************************************************/
static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
    platform_device_unregister(&HardwareInfo_device);
}
/*****************************************************************************/
module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
/*****************************************************************************/
MODULE_AUTHOR("Kaka Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("MT6575 Hareware Info driver");
MODULE_LICENSE("GPL");



