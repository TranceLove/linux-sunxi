/* 
 * drivers/input/touchscreen/zforce0x_ts.c
 *
 * FocalTech zforce TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *  for this touchscreen to work, it's slave addr must be set to 0x7e | 0x70
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include "zforce_ts.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>
#include "ctp_platform_ops.h"

#define FOR_TSLIB_TEST
//#define PRINT_INT_INFO
//#define PRINT_POINT_INFO
//#define DEBUG
#define TOUCH_KEY_SUPPORT
#ifdef TOUCH_KEY_SUPPORT
#define TOUCH_KEY_LIGHT_SUPPORT
#define TOUCH_KEY_FOR_EVB13
#define TOUCH_KEY_FOR_ANGDA
#ifdef TOUCH_KEY_FOR_ANGDA
#define TOUCH_KEY_X_LIMIT	(60000)
//#define TOUCH_KEY_NUMBER	(4)
#endif
#ifdef TOUCH_KEY_FOR_EVB13
#define TOUCH_KEY_LOWER_X_LIMIT	(848)
#define TOUCH_KEY_HIGHER_X_LIMIT	(852)
//#define TOUCH_KEY_NUMBER	(5)
#endif
#endif

//#define CONFIG_SUPPORT_FTS_CTP_UPG


struct i2c_dev{
struct list_head list;
struct i2c_adapter *adap;
struct device *dev;
};

static struct class *i2c_dev_class;
static LIST_HEAD (i2c_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);

static int ctp_vendor;
static int ctp_a730_id = 0;
static int key_value = 0;

static struct i2c_client *this_client;
#ifdef TOUCH_KEY_LIGHT_SUPPORT
static int gpio_light_hdle = 0;
#endif
#ifdef TOUCH_KEY_SUPPORT
static int key_tp  = 0;
static int key_val = 0;
#endif

#ifdef PRINT_POINT_INFO
#define print_point_info(fmt, args...)   \
        do{                              \
                pr_info(fmt, ##args);     \
        }while(0)
#else
#define print_point_info(fmt, args...)   //
#endif

#ifdef PRINT_INT_INFO
#define print_int_info(fmt, args...)     \
        do{                              \
                pr_info(fmt, ##args);     \
        }while(0)
#else
#define print_int_info(fmt, args...)   //
#endif
///////////////////////////////////////////////
//specific tp related macro: need be configured for specific tp
#define CTP_IRQ_NO			(gpio_int_info[0].port_num)

#define CTP_IRQ_MODE			(LOW_LEVEL)
#define CTP_NAME		"zforce"	
#define TS_RESET_LOW_PERIOD		(1)
#define TS_INITIAL_HIGH_PERIOD		(30)
#define TS_WAKEUP_LOW_PERIOD	(20)
#define TS_WAKEUP_HIGH_PERIOD	(20)
#define TS_POLL_DELAY			(10)	/* ms delay between samples */
#define TS_POLL_PERIOD			(10)	/* ms delay between samples */
#define SCREEN_MAX_X			(screen_max_x)
#define SCREEN_MAX_Y			(screen_max_y)
#define PRESS_MAX			(255)


#ifndef CONFIG_HAS_EARLYSUSPEND
#define CONFIG_HAS_EARLYSUSPEND
#endif


static void* __iomem gpio_addr = NULL;
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_reset_hdle = 0;
static int gpio_wakeup_enable = 1;
static int gpio_reset_enable = 1;
static user_gpio_set_t gpio_int_info[1];


static int screen_max_x = 0;
static int screen_max_y = 0;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
/* Addresses to scan */
static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};
static __u32 twi_id = 0;

/*
 * ctp_get_pendown_state  : get the int_line data state,
 *
 * return value:
 *             return PRESS_DOWN: if down
 *             return FREE_UP: if up,
 *             return 0: do not need process, equal free up.
 */
static int ctp_get_pendown_state(void)
{
	unsigned int reg_val;
	static int state = FREE_UP;

	//get the input port state
	reg_val = readl(gpio_addr + PIOH_DATA);
	//pr_info("reg_val = %x\n",reg_val);
	if(!(reg_val & (1<<CTP_IRQ_NO))){
		state = PRESS_DOWN;
		print_int_info("pen down. \n");
	}else{ //touch panel is free up
		state = FREE_UP;
		print_int_info("free up. \n");
	}
	return state;
}

/**
 * ctp_clear_penirq - clear int pending
 *
 */
static void ctp_clear_penirq(void)
{
	int reg_val;
	//clear the IRQ_EINT29 interrupt pending
	//pr_info("clear pend irq pending\n");
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	//writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	//writel(reg_val&(1<<(IRQ_EINT21)),gpio_addr + PIO_INT_STAT_OFFSET);
	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		print_int_info("==CTP_IRQ_NO=\n");
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	return;
}

/**
 * ctp_set_irq_mode - according sysconfig's subkey "ctp_int_port" to config int port.
 *
 * return value:
 *              0:      success;
 *              others: fail;
 */
static int ctp_set_irq_mode(char *major_key, char *subkey, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	pr_info("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		pr_info("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
	gpio_get_one_pin_status(gpio_int_hdle, gpio_int_info, subkey, 1);
	pr_info("%s, %d: gpio_int_info, port = %d, port_num = %d. \n", __func__, __LINE__, \
		gpio_int_info[0].port, gpio_int_info[0].port_num);
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	pr_info(" INTERRUPT CONFIG\n");
	reg_num = (gpio_int_info[0].port_num)%8;
	reg_addr = (gpio_int_info[0].port_num)/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();

	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET);
	reg_val |= (1 << (gpio_int_info[0].port_num));
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;
}

/**
 * ctp_set_gpio_mode - according sysconfig's subkey "ctp_io_port" to config io port.
 *
 * return value:
 *              0:      success;
 *              others: fail;
 */
static int ctp_set_gpio_mode(void)
{
	//int reg_val;
	int ret = 0;
	//config gpio to io mode
	pr_info("%s: config gpio to io mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
	if(!gpio_int_hdle){
		pr_info("request ctp_io_port failed. \n");
		ret = -1;
		goto request_tp_io_port_failed;
	}
#endif
	return ret;

request_tp_io_port_failed:
	return ret;
}

/**
 * ctp_judge_int_occur - whether interrupt occur.
 *
 * return value:
 *              0:      int occur;
 *              others: no int occur;
 */
static int ctp_judge_int_occur(void)
{
	//int reg_val[3];
	int reg_val;
	int ret = -1;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO))){
		ret = 0;
	}
	return ret;
}

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 *
 */
static void ctp_free_platform_resource(void)
{
	if(gpio_addr){
		iounmap(gpio_addr);
	}

	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}

	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}

	if(gpio_reset_hdle){
		gpio_release(gpio_reset_hdle, 2);
	}

	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 *
 */
static int ctp_init_platform_resource(void)
{
	int ret = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	//pr_info("%s, gpio_addr = 0x%x. \n", __func__, gpio_addr);
	if(!gpio_addr) {
		ret = -EIO;
		goto exit_ioremap_failed;
	}
	//    gpio_wakeup_enable = 1;
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		pr_warning("%s: tp_wakeup request gpio fail!\n", __func__);
		gpio_wakeup_enable = 0;
	}

	gpio_reset_hdle = gpio_request_ex("ctp_para", "ctp_reset");
	if(!gpio_reset_hdle) {
		pr_warning("%s: tp_reset request gpio fail!\n", __func__);
		gpio_reset_enable = 0;
	}

	return ret;

exit_ioremap_failed:
	ctp_free_platform_resource();
	return ret;
}


/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	__u32 twi_addr = 0;
	//__u32 twi_id = 0;
	script_parser_value_type_t type = SCRIPT_PARSER_VALUE_TYPE_STRING;

	pr_info("%s. \n", __func__);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(1 != ctp_used){
		pr_err("%s: ctp_unused. \n",  __func__);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	if(strcmp(CTP_NAME, name)){
		pr_err("%s: name %s does not match CTP_NAME. \n", __func__, name);
		pr_err(CTP_NAME);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	//big-endian or small-endian?
	//pr_info("%s: before: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	pr_info("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//pr_info("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		goto script_parser_fetch_err;
	}
	pr_info("%s: ctp_twi_id is %d. \n", __func__, twi_id);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_x = %d. \n", __func__, screen_max_x);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_y = %d. \n", __func__, screen_max_y);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		goto script_parser_fetch_err;
	}
	pr_info("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);

	return 0;

script_parser_fetch_err:
	pr_notice("=========script_parser_fetch_err============\n");
	return ret;
}

/**
 * ctp_reset - function
 *
 */
static void ctp_reset(void)
{
	if(gpio_reset_enable){
		pr_info("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 0, "ctp_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_RESET_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "ctp_reset")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_INITIAL_HIGH_PERIOD);
	}
}

/**
 * ctp_wakeup - function
 *
 */
static void ctp_wakeup(void)
{

	if(1 == gpio_wakeup_enable){
		pr_info("%s. \n", __func__);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			pr_info("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_HIGH_PERIOD);

	}

	return;
}
/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}
////////////////////////////////////////////////////////////////

static struct ctp_platform_ops ctp_ops = {
	.get_pendown_state = ctp_get_pendown_state,
	.clear_penirq	   = ctp_clear_penirq,
	.set_irq_mode      = ctp_set_irq_mode,
	.set_gpio_mode     = ctp_set_gpio_mode,
	.judge_int_occur   = ctp_judge_int_occur,
	.init_platform_resource = ctp_init_platform_resource,
	.free_platform_resource = ctp_free_platform_resource,
	.fetch_sysconfig_para = ctp_fetch_sysconfig_para,
	.ts_reset =          ctp_reset,
	.ts_wakeup =         ctp_wakeup,
	.ts_detect = ctp_detect,
};

int fts_ctpm_fw_upgrade_with_i_file(void);

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	spin_lock(&i2c_dev_list_lock);

	list_for_each_entry(i2c_dev,&i2c_dev_list,list){
		pr_info("--line = %d ,i2c_dev->adapt->nr = %d,index = %d.\n",__LINE__,i2c_dev->adap->nr,index);
		if(i2c_dev->adap->nr == index){
		     goto found;
		}
	}
	i2c_dev = NULL;

found:
	spin_unlock(&i2c_dev_list_lock);

	return i2c_dev ;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS){
		pr_info("i2c-dev:out of device minors (%d) \n",adap->nr);
		return ERR_PTR (-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev){
		return ERR_PTR(-ENOMEM);
	}
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);

	return i2c_dev;
}


static int zforce_i2c_rxdata(char *rxdata, int length);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
	s16 touch_ID1;
	s16 touch_ID2;
	s16 touch_ID3;
	s16 touch_ID4;
	s16 touch_ID5;
    u8  touch_point;
    u8  gest_id;
};

struct zforce_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
/*#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif*/
};

/* ---------------------------------------------------------------------
*
*   Focal Touch panel upgrade related driver
*
*
----------------------------------------------------------------------*/

typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS        (0x70>>1)

void delay_ms(FTS_WORD  w_ms)
{
	//platform related, please implement this function
	msleep( w_ms );
}


/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_read_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;

	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret != dw_lenth){
		pr_info("ret = %d. \n", ret);
		pr_info("i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
int i2c_write_interface(u8 bt_ctpm_addr, u8* pbt_buf, u16 dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret != dw_lenth){
		pr_info("i2c_write_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}


/***************************************************************************************/

/*
[function]:
    read out the register value.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[out]    :the returned register value;
    bt_len[in]        :length of pbt_buf, should be set to 2;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
u8 fts_register_read(u8 e_reg_name, u8* pbt_buf, u8 bt_len)
{
	u8 read_cmd[3]= {0};
	u8 cmd_len     = 0;

	read_cmd[0] = e_reg_name;
	cmd_len = 1;

	/*call the write callback function*/
	//    if(!i2c_write_interface(I2C_CTPM_ADDRESS, &read_cmd, cmd_len))
	//    {
	//        return FTS_FALSE;
	//    }


	if(!i2c_write_interface(I2C_CTPM_ADDRESS, read_cmd, cmd_len))	{//change by zhengdixu
		return FTS_FALSE;
	}

	/*call the read callback function to get the register value*/
	if(!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len)){
		return FTS_FALSE;
	}
	return FTS_TRUE;
}

/*
[function]:
    write a value to register.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[in]        :the returned register value;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int fts_register_write(u8 e_reg_name, u8 bt_value)
{
	FTS_BYTE write_cmd[2] = {0};

	write_cmd[0] = e_reg_name;
	write_cmd[1] = bt_value;

	/*call the write callback function*/
	//return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, 2);
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, 2); //change by zhengdixu
}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	//return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, num);
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);//change by zhengdixu
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_write(u8* pbt_buf, u16 dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
int byte_read(u8* pbt_buf, u8 bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
	//zforce_i2c_rxdata
}


/*
[function]:
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH       128 //2//4//8//16//32//64//128//256

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};
unsigned char fts_ctpm_get_i_file_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(u8* pbt_buf, u16 dw_lenth)
{
    u8 reg_val[2] = {0};
    FTS_BOOL i_ret = 0;
    u16 i = 0;


    u16  packet_number;
    u16  j;
    u16  temp;
    u16  lenght;
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8  auc_i2c_write_buf[10];
    u8 bt_ecc;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    delay_ms(100);
    fts_register_write(0xfc,0xaa);
    delay_ms(50);
     /*write 0x55 to register 0xfc*/
    fts_register_write(0xfc,0x55);
    pr_info("Step 1: Reset CTPM test\n");

    delay_ms(30);

    /*********Step 2:Enter upgrade mode *****/
     auc_i2c_write_buf[0] = 0x55;
     auc_i2c_write_buf[1] = 0xaa;
     i = 0;
     do{
        i++;
        i_ret = i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
        pr_info("Step 2: Enter update mode. \n");
        delay_ms(5);
     }while((FTS_FALSE == i_ret) && i<5);

    /*********Step 3:check READ-ID***********************/
    /*send the opration head*/
    i = 0;
    do{
        if(i > 3)
        {
          cmd_write(0x07,0x00,0x00,0x00,1);
		  return ERR_READID;
        }
        /*read out the CTPM ID*/
        pr_info("====Step 3:check READ-ID====");
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);
        i++;
        delay_ms(5);
        pr_info("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }while(reg_val[1] != 0x03);//while(reg_val[0] != 0x79 || reg_val[1] != 0x03);

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);
    delay_ms(1500);
    pr_info("Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    pr_info("Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              pr_info("upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        delay_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        delay_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    //cmd_write(0xcc,0x00,0x00,0x00,1);//0xcc as register address and read 1 byte
   // byte_read(reg_val,1);//change by zhengdixu

	fts_register_read(0xcc, reg_val,1);

    pr_info("Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
       cmd_write(0x07,0x00,0x00,0x00,1);
		return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);
    msleep(30);
    return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    pr_info("[FTS] start auto CLB.\n");
    msleep(200);
    fts_register_write(0, 0x40);
    delay_ms(100);                       //make sure already enter factory mode
    fts_register_write(2, 0x4);               //write command to start calibration
    delay_ms(300);
    for(i=0;i<100;i++)
    {
        fts_register_read(0,&uc_temp,1);
        if ( ((uc_temp&0x70)>>4) == 0x0)    //return to normal mode, calibration finish
        {
            break;
        }
        delay_ms(200);
        pr_info("[FTS] waiting calibration %d\n",i);
    }

    pr_info("[FTS] calibration OK.\n");

    msleep(300);
    fts_register_write(0, 0x40);              //goto factory mode
    delay_ms(100);                       //make sure already enter factory mode
   fts_register_write(2, 0x5);               //store CLB result
    delay_ms(300);
    fts_register_write(0, 0x0);               //return to normal mode
    msleep(300);
    pr_info("[FTS] store CLB result OK.\n");
    return 0;
}
void getVerNo(u8* buf, int len)
{
	u8 start_reg=0x0;
	int ret = -1;
	//int status = 0;
	int i = 0;
	start_reg = 0xa6;

#if 0
	pr_info("read 0xa6 one time. \n");
	if(FTS_FALSE == fts_register_read(0xa6, buf, len)){
        return ;
	}

	for (i=0; i< len; i++)
	{
		pr_info("=========buf[%d] = 0x%x \n", i, buf[i]);
	}

	pr_info("read 0xa8. \n");
	if(FTS_FALSE == fts_register_read(0xa8, buf, len)){
        return ;
	}
	for (i=0; i< len; i++)
	{
		pr_info("=========buf[%d] = 0x%x \n", i, buf[i]);
	}

	zforce_i2c_rxdata(buf, len);

    for (i=0; i< len; i++)
        {
            pr_info("=========buf[%d] = 0x%x \n", i, buf[i]);
        }

    byte_read(buf, len);
    for (i=0; i< len; i++)
    {
        pr_info("=========buf[%d] = 0x%x \n", i, buf[i]);
    }

#endif

	ret =fts_register_read(0xa6, buf, len);
	//et = ft5406_read_regs(zforce0x_ts_data_test->client,start_reg, buf, 2);
	if (ret < 0)
	{
		pr_info("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return;
	}
	for (i=0; i<2; i++)
	{
		pr_info("=========buf[%d] = 0x%x \n", i, buf[i]);
	}


	return;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret = 0;
	unsigned char a;
	unsigned char b;
#define BUFFER_LEN (2)            //len == 2
	unsigned char buf[BUFFER_LEN] = {0};

	//=========FW upgrade========================*/
	pr_info("%s. \n", __func__);

	pbt_buf = CTPM_FW;
	//msleep(200);
   // cmd_write(0x07,0x00,0x00,0x00,1);
	msleep(100);
	getVerNo(buf, BUFFER_LEN);
	a = buf[0];
	b = fts_ctpm_get_i_file_ver();
	pr_info("a == %hu,  b== %hu \n",a, b);

	/*
	  * when the firmware in touch panel maybe corrupted,
	  * or the firmware in host flash is new, need upgrade
	  */
	if ( 0xa6 == a ||a < b ){
		/*call the upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(&pbt_buf[0],sizeof(CTPM_FW));
		if (i_ret != 0){
			pr_info("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		} else {
			pr_info("[FTS] upgrade successfully.\n");
			fts_ctpm_auto_clb();  //start auto CLB
		}

	}

	return i_ret;

}

unsigned char fts_ctpm_get_upg_ver(void)
{
	unsigned int ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2){
		return CTPM_FW[0];
	}
	else{
		//TBD, error handling?
		return 0xff; //default value
	}
}

static int zforce_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

        //pr_info("IIC add = %x\n",this_client->addr);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_info("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int zforce_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int zforce_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = zforce_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static void zforce_get_reg_key(u8 gest_id)
{
	//broncho xiangyu
	if(ctp_vendor == 1)
	{
		switch(gest_id)
		{
			case 1:
				key_value = KEY_MENU;
				break;
			case 2:
				key_value = KEY_HOME;
				break;
			case 4:
				key_value = KEY_BACK;
				break;
			case 8:
				key_value = KEY_SEARCH;
				break;
			default:
				break;
		}
	}
	else if(ctp_vendor == 2) //a723b
	{
		switch(gest_id)
		{
			case 1:
				key_value = KEY_BACK;
				break;
			case 2:
				key_value = KEY_HOME;
				break;
			case 4:
				key_value = KEY_MENU;
				break;
			case 8:
				key_value = KEY_SEARCH;
				break;
			default:
				break;
		}
	}
	else
	{
	}

	return ;//broncho xiangyu
}

static void zforce_report_key(u16 x, u16 y)
{
	return ;
}

static void zforce_ts_release(void)
{
	struct zforce_ts_data *data = i2c_get_clientdata(this_client);

#ifdef CONFIG_FT5X0X_MULTITOUCH	
#ifdef TOUCH_KEY_SUPPORT
	if(1 == key_tp) {
		input_report_key(data->input_dev, key_val, 0);
		print_point_info("Release key_val = %d\n",key_val);		
	} else if(key_value != 0) {
		input_report_key(data->input_dev, key_value, 0);
		print_point_info("Release key_value = %d\n",key_value);		
		key_value = 0;
	} else {
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	}
#else
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#endif

#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif

	input_sync(data->input_dev);
	return;

}

static int zforce_read_data(void)
{
	struct zforce_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	unsigned char buf[32]={0};
	int ret = -1;

	ret = zforce_i2c_rxdata(buf, 31);

	if (ret < 0) {
		pr_info("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	//memset(event, 0, sizeof(struct ts_event));

	event->touch_point = buf[2] & 0x07;// 000 0111
	event->gest_id = buf[1];

#ifdef PRINT_POINT_INFO
	event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
	event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	printk("touch point = %d\n",event->touch_point);
	printk("gest_id = %d\n",event->gest_id);
	printk("event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
#endif

	if ((event->touch_point == 0) && (event->gest_id == 0)) {
		zforce_ts_release();
		return 1;
	}


#ifdef CONFIG_FT5X0X_MULTITOUCH
	if(event->gest_id != 0)
	{
		zforce_get_reg_key(event->gest_id);
		print_point_info("after get key_value = %d\n", key_value);
	}
	else
	{
		switch (event->touch_point) {
			case 5:
				event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
				event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
				if(1 == exchange_x_y_flag){
					swap(event->x5, event->y5);
				}
				//pr_info("after swap: event->x5 = %d, event->y5 = %d. \n", event->x5, event->y5);

				if(1 == revert_x_flag){
					event->x5 = SCREEN_MAX_X - event->x5;
				}
				if(1 == revert_y_flag){
					event->y5 = SCREEN_MAX_Y - event->y5;
				}
				//pr_info("before swap: event->x5 = %d, event->y5 = %d. \n", event->x5, event->y5);
				event->touch_ID5=(s16)(buf[0x1D] & 0xF0)>>4;
			case 4:
				event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
				event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
				if(1 == exchange_x_y_flag){
					swap(event->x4, event->y4);
				}
				//pr_info("after swap: event->x4 = %d, event->y4 = %d. \n", event->x4, event->y4);

				if(1 == revert_x_flag){
					event->x4 = SCREEN_MAX_X - event->x4;
				}
				if(1 == revert_y_flag){
					event->y4 = SCREEN_MAX_Y - event->y4;
				}
				//pr_info("before swap: event->x4 = %d, event->y4 = %d. \n", event->x4, event->y4);
				event->touch_ID4=(s16)(buf[0x17] & 0xF0)>>4;
			case 3:
				event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
				event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
				if(1 == exchange_x_y_flag){
					swap(event->x3, event->y3);
				}
				//pr_info("after swap: event->x3 = %d, event->y3 = %d. \n", event->x3, event->y3);

				if(1 == revert_x_flag){
					event->x3 = SCREEN_MAX_X - event->x3;
				}
				if(1 == revert_y_flag){
					event->y3 = SCREEN_MAX_Y - event->y3;
				}
				//pr_info("before swap: event->x3 = %d, event->y3 = %d. \n", event->x3, event->y3);
				event->touch_ID3=(s16)(buf[0x11] & 0xF0)>>4;
			case 2:
				event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
				event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
				if(1 == exchange_x_y_flag){
					swap(event->x2, event->y2);
				}
				//pr_info("after swap: event->x2 = %d, event->y2 = %d. \n", event->x2, event->y2);

				if(1 == revert_x_flag){
					event->x2 = SCREEN_MAX_X - event->x2;
				}
				if(1 == revert_y_flag){
					event->y2 = SCREEN_MAX_Y - event->y2;
				}
				//pr_info("before swap: event->x2 = %d, event->y2 = %d. \n", event->x2, event->y2);
				event->touch_ID2=(s16)(buf[0x0b] & 0xF0)>>4;
			case 1:
				event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
				event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
#ifdef TOUCH_KEY_FOR_ANGDA
				if(event->x1 < TOUCH_KEY_X_LIMIT)
				{
					if(1 == exchange_x_y_flag){
						swap(event->x1, event->y1);
					}

					if(1 == revert_x_flag){
						event->x1 = SCREEN_MAX_X - event->x1;
					}
					if(1 == revert_y_flag){
						event->y1 = SCREEN_MAX_Y - event->y1;
					}
					//pr_info("before swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
				}
#elif defined(TOUCH_KEY_FOR_EVB13)
				if((event->x1 > TOUCH_KEY_LOWER_X_LIMIT)&&(event->x1<TOUCH_KEY_HIGHER_X_LIMIT))
				{
					if(1 == revert_x_flag){
						event->x1 = SCREEN_MAX_X - event->x1;
					}
					if(1 == revert_y_flag){
						event->y1 = SCREEN_MAX_Y - event->y1;
					}
					//pr_info("before swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
					if(1 == exchange_x_y_flag){
						swap(event->x1, event->y1);
					}
				}
#else
				if(event->x1 < SCREEN_MAX_X)            //if key point do not change
				{
					if(1 == exchange_x_y_flag){
						swap(event->x1, event->y1);
					}
					//printk("after swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
					if(1 == revert_x_flag){
						event->x1 = SCREEN_MAX_X - event->x1;
					}
					if(1 == revert_y_flag){
						event->y1 = SCREEN_MAX_Y - event->y1;
					}
					//pr_info("after revert y: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
				}
#endif

				//pr_info("after swap: event->x1 = %d, event->y1 = %d. \n", event->x1, event->y1);
				event->touch_ID1=(s16)(buf[0x05] & 0xF0)>>4;
				break;
			default:
				return -1;
		}
	}
#else
	if (event->touch_point == 1) {
		event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	}
#endif
	event->pressure = 200;

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d\n", __func__,
			event->x1, event->y1, event->x2, event->y2);


    return 0;
}

#ifdef TOUCH_KEY_LIGHT_SUPPORT
static void zforce_lighting(void)
{
	if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_light_hdle, 1, "ctp_light")){
		pr_info("zforce_ts_light: err when operate gpio. \n");
	}
	msleep(15);
	if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_light_hdle, 0, "ctp_light")){
		pr_info("zforce_ts_light: err when operate gpio. \n");
	}

	return;
}
#endif

#ifdef TOUCH_KEY_SUPPORT
static void zforce_report_touchkey(void)
{
	struct zforce_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	//print_point_info("x=%d===Y=%d\n",event->x1,event->y1);

	if(event->gest_id != 0) //report key
	{
#ifdef PRINT_POINT_INFO
		printk("gest_id key_value %d\n", key_value);
#endif
		input_report_key(data->input_dev, key_value, 1);
		input_sync(data->input_dev);  
	}
	else //report x y
	{
#ifdef TOUCH_KEY_FOR_ANGDA
		if((1==event->touch_point)&&(event->x1 > TOUCH_KEY_X_LIMIT)){
			key_tp = 1;
			if(event->y1 < 40){
				key_val = 1;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);  
				print_point_info("===KEY 1====\n");
			}else if(event->y1 < 90){
				key_val = 2;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);     
				print_point_info("===KEY 2 ====\n");
			}else{
				key_val = 3;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);     
				print_point_info("===KEY 3====\n");	
			}
		} else{
			key_tp = 0;
		}
#endif
#ifdef TOUCH_KEY_FOR_EVB13
		if((1==event->touch_point)&&((event->x1 > TOUCH_KEY_LOWER_X_LIMIT)&&(event->x1<TOUCH_KEY_HIGHER_X_LIMIT))){
			key_tp = 1;
			if(event->y1 < 5){
				key_val = 1;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);
				print_point_info("===KEY 1====\n");
			}else if((event->y1 < 45)&&(event->y1>35)){
				key_val = 2;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);
				print_point_info("===KEY 2 ====\n");
			}else if((event->y1 < 75)&&(event->y1>65)){
				key_val = 3;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);
				print_point_info("===KEY 3====\n");
			}else if ((event->y1 < 105)&&(event->y1>95))	{
				key_val = 4;
				input_report_key(data->input_dev, key_val, 1);
				input_sync(data->input_dev);
				print_point_info("===KEY 4====\n");
			}
		}else{
			key_tp = 0;
		}
#else
		if((1==event->touch_point)&&((event->x1 > SCREEN_MAX_X + 10) || (event->y1 > SCREEN_MAX_Y + 10)))
		{
			print_point_info("event->y1 %d\n", event->y1);
			zforce_report_key(event->x1, event->y1);
			print_point_info("key_value =%d\n", key_value);
			input_report_key(data->input_dev, key_value, 1);
			input_sync(data->input_dev);     
		}
#endif
	}
#ifdef TOUCH_KEY_LIGHT_SUPPORT
	zforce_lighting();
#endif
	return;
}
#endif

#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
static void zforce_ts_suspend(struct early_suspend *handler)
{
	struct zforce_ts_data *ts;
	ts =  container_of(handler, struct zforce_ts_data, early_suspend);
	
	



}

static void zforce_ts_resume(struct early_suspend *handler)
{
	pr_info("==zforce_ts_resume== \n");
	//reset
	ctp_ops.ts_reset();
	//wakeup
	ctp_ops.ts_wakeup();
}
#endif  //CONFIG_HAS_EARLYSUSPEND
#endif


static int detect_device(void)
{
	return 0;
} 

#ifndef   CONFIG_FT5X0X_MULTOUCH
#define   CONFIG_FT5X0X_MULTOUCH
#endif


#define ZF_DEACTIVATE 0x00
#define ZF_ACTIVATE 0x01
#define ZF_SET_RESOLUTION 0x02
#define ZF_CONFIGURE 0x03
#define ZF_REQ_COORDINATES 0x04
#define ZF_SCANNINGFREN	   0x08
#define ZF_AREA_SIZE 0x09
#define ZF_REQ_VERSION 0x0A
#define ZF_CALIBRATION 0x1A
// Bit flags for the startup state machinery. Response flags are set at response from the
// ZForce, but ignored by the program logic

#define   ZF_VERSION_REQUESTED       0x0001
#define   ZF_VERSION_RESPONDED       0x0002
#define   ZF_DEACTIVATE_REQUESTED    0x0004
#define   ZF_DEACTIVATE_RESPONDED    0x0008
#define   ZF_ACTIVATE_REQUESTED  	 0x0010
#define   ZF_ACTIVATE_RESPONDED   	 0x0020
#define   ZF_RESOLUTION_REQUESTED	 0x0040
#define   ZF_RESOLUTION_RESPONDED	 0x0080
#define   ZF_CONFIGURE_REQUESTED 	 0x0100
#define   ZF_CONFIGURE_RESPONDED 	 0x0200
#define   ZF_FIRSTTOUCH_REQUESTED	 0x0400
#define   ZF_FIRSTTOUCH_RESPONDED	 0x0800

//add by Ethan
#define   ZF_TOUCHAREA_REQUESTED 	 0X1000
#define   ZF_TOUCHAREA_RESPONDED 	 0x2000
#define   ZF_CALIBRATION_REQUESTED	 0x4000
#define   ZF_CALIBRATION_RESPONDED   0x8000
//end add


#define ZF_MAX_X 800
#define ZF_MAX_Y 480

// ZForce-device data
struct zforce_ts_priv {
	struct i2c_client *client;  // I2C client to communicate with the ZF chip
	struct input_dev *input;    // The registered input device to which events are reported

//	struct delayed_work work;   // Scheduler entry for ISR "bottom half"
	struct work_struct  work;
/*	
	struct early_suspend early_suspend;  //zforce chip sleep/wake up
*/
	int irq;   // IRQ on which ZF signals data available
	int startup_state; // Bit flags according to ZF_xx_REQUESTED/RESPONDED definitions
};


static void zforce_ts_handle_deactivate(u_int8_t *data, int len);
static void zforce_ts_handle_activate(u_int8_t *data, int len);
static void zforce_ts_handle_configure(u_int8_t *data, int len);
static void zforce_ts_handle_setres(u_int8_t *data, int len);
static void zforce_ts_handle_touchdata(struct zforce_ts_priv *priv,
		u_int8_t *data, int len);
static void zforce_ts_handle_version(u_int8_t *data, int len);
static int zforce_ts_send_deactivate(struct zforce_ts_priv *priv);
static int zforce_ts_send_activate(struct zforce_ts_priv *priv);
static int zforce_ts_send_configure(struct zforce_ts_priv *priv, int dual_touch);
static int zforce_ts_send_setres(struct zforce_ts_priv *priv, int width,
		int height);
static int zforce_ts_send_touchdata_req(struct zforce_ts_priv *priv);
static int zforce_ts_send_version_req(struct zforce_ts_priv *priv);


//add by Ethan

static int zforce_ts_send_calibration(struct zforce_ts_priv *priv);
static void zforce_ts_handle_calibration(u_int8_t *data, int len);
static void zforce_ts_handle_toucharea(u_int8_t *data, int len);
static int zforce_ts_send_areasize(struct zforce_ts_priv *priv);

//end add


/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static void zforce_ts_suspend(struct early_suspend *handler)
{
    struct zforce_ts_priv  *priv; 
    priv =  container_of(handler, struct zforce_ts_priv, early_suspend);
    pr_info("==zforce_ts_suspend");
    zforce_ts_send_deactivate(priv);
	return;


}

static void zforce_ts_resume(struct early_suspend *handler)
{
    struct zforce_ts_priv *priv;
	priv = container_of(handler, struct zforce_ts_priv, early_suspend);
	pr_info("==zforce_ts_resume== \n");

    //wakeup
    ctp_wakeup();
   // mdelay(50);
	zforce_ts_send_activate(priv);
	return;
	
}
#endif  //CONFIG_HAS_EARLYSUSPEND
*/





static void zforce_ts_handle_data(struct work_struct *work) {

	// The container struct of the work_struct passed to the schedule call happens to be
	// the private data associated with this driver, use the container_of
	// macro to acquire it
	struct zforce_ts_priv *priv = container_of(work, struct zforce_ts_priv,work);// work.work);
	// Current command received
	int zforce_command;
	// Length of payload part of received block
	int payload_length;
	int j;
	int reg_val;
	// Data buffer (large enough to hold biggest expected block)
	u_int8_t tmp_start[2];
	u_int8_t tmp_buf[20];
	int more_verbose;
	int i;

	memset(tmp_buf, 0, sizeof(tmp_buf));

	// Be more verbose until one touch coordinate was handled
	more_verbose = priv->startup_state & ZF_FIRSTTOUCH_RESPONDED ? 0 : 1;

	// Read the first three bytes to get the command id and the size of the rest of the message
	if (i2c_master_recv(priv->client, tmp_start, 2) != 2) {
		dev_err(&priv->client->dev, "Unable to read ZForce data header\n");
		goto out;
	}

	// Check the start byte
	if (tmp_start[0] != 0xee) {
		dev_err(&priv->client->dev,
				"Invalid initial byte of ZForce data block\n");
		goto out;
	}

	// Get the length of the payload
	payload_length = tmp_start[1] ;

	// Get the command
	//  zforce_command = tmp_buf[2];	
	// The block is too long to handle
	if (payload_length > sizeof(tmp_buf)) {

		dev_err(&priv->client->dev, "Block from Zforce was too long\n");

		// Read byte by byte to flush the buffer
		//TODO: Read bigger blocks, set a max limit, maybe try to resync after this,
		//  at multiple failure, the ISR shoud be switched off

		for (i = 0; i < payload_length; i++) {

			i2c_master_recv(priv->client, tmp_buf, 1);
		}
		//modify
		//schedule_delayed_work(&priv->work, HZ / 20);
		schedule_work(&priv->work);
		return;
	}

	// Reuse the data buffer and read the payload part of the block
	if (i2c_master_recv(priv->client, tmp_buf, payload_length)
			!= payload_length) {
		dev_err(&priv->client->dev, "Unable to get ZForce data header\n");
		goto out;
	}

	zforce_command = tmp_buf[0];
	/***************/
	//printk("Data from ZForce, command=%d, length=%d\n", zforce_command, payload_length);
	printk("recieved:  0x%x ,  0x%x",tmp_start[0], tmp_start[1]);
	for(j = 0; j < payload_length; j++){

	printk("  0x%x, \t",tmp_buf[j]);
	}
	printk("\n");
	/***************************/
	switch (zforce_command) { // Attend to the command

		case 0x07:
			priv->startup_state = 0;
			break;

		case ZF_DEACTIVATE: // Got response from deactivate request
			priv->startup_state |= ZF_DEACTIVATE_RESPONDED;
			zforce_ts_handle_deactivate(tmp_buf, payload_length);
			break;

		case ZF_ACTIVATE: // Got response from activate request
			priv->startup_state |= ZF_ACTIVATE_RESPONDED;
			zforce_ts_handle_activate(tmp_buf, payload_length);
			break;

		case ZF_CONFIGURE: // Got response from configuration request
			priv->startup_state |= ZF_CONFIGURE_RESPONDED;
			zforce_ts_handle_configure(tmp_buf, payload_length);
			break;

		case ZF_SET_RESOLUTION: // Got response from resolution setting request
			priv->startup_state |= ZF_RESOLUTION_RESPONDED;
			zforce_ts_handle_setres(tmp_buf, payload_length);
			break;

		case ZF_REQ_COORDINATES: // Got touch event
			//		printk("----------------------enter coordinates-------------------");
			zforce_ts_handle_touchdata(priv, tmp_buf +1, payload_length - 1);
			priv->startup_state |= ZF_FIRSTTOUCH_RESPONDED;
			break;

		case ZF_REQ_VERSION: // Got version
			priv->startup_state |= ZF_VERSION_RESPONDED;
			zforce_ts_handle_version(tmp_buf, payload_length);
			break;
		
		case ZF_AREA_SIZE: //set touch area size
			priv->startup_state |=  ZF_TOUCHAREA_RESPONDED;
			zforce_ts_handle_toucharea(tmp_buf, payload_length);
			break;
		
		case ZF_CALIBRATION:
			priv->startup_state |= ZF_CALIBRATION_RESPONDED;
			zforce_ts_handle_calibration(tmp_buf, payload_length);
			break;
	}

	if (!(priv->startup_state & ZF_DEACTIVATE_REQUESTED)) {
		// Still not deactivated, send a deactivation request, this is
		// necessary as the ZForce will potentially stop sending coordinates
		// if an activation is issued without prior deactivation
		priv->startup_state |= ZF_DEACTIVATE_REQUESTED;
		zforce_ts_send_deactivate(priv);
	} else if (!(priv->startup_state & ZF_ACTIVATE_REQUESTED)) {
		// Still not activated, send an activation request
		priv->startup_state |= ZF_ACTIVATE_REQUESTED;
		zforce_ts_send_activate(priv);
	} else if (!(priv->startup_state & ZF_RESOLUTION_REQUESTED)) {
		// Still no resolution set, send a resolution setting request
		priv->startup_state |= ZF_RESOLUTION_REQUESTED;
		zforce_ts_send_setres(priv, ZF_MAX_X, ZF_MAX_Y);
	} else if (!(priv->startup_state & ZF_CONFIGURE_REQUESTED)) {
		// Still not configured, send a configuration request
		priv->startup_state |= ZF_CONFIGURE_REQUESTED;
		zforce_ts_send_configure(priv, 1);
	} else if (!(priv->startup_state & ZF_TOUCHAREA_REQUESTED)){
		priv->startup_state |= ZF_TOUCHAREA_REQUESTED;
		zforce_ts_send_areasize(priv);
	} else if (!(priv->startup_state & ZF_CALIBRATION_REQUESTED)){
		priv->startup_state |= ZF_CALIBRATION_REQUESTED;
		zforce_ts_send_calibration(priv);
	} else if (!(priv->startup_state & ZF_FIRSTTOUCH_REQUESTED)){
		// All setup done, request some coordinates
		priv->startup_state |= ZF_FIRSTTOUCH_REQUESTED;
		zforce_ts_send_touchdata_req(priv);
	}

out:
	//printk("-----------------------------%d -----------------\n",__LINE__);
	//	schedule_delayed_work(&priv->work, HZ / 20);
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	writel(reg_val&(1<<(IRQ_EINT21)),gpio_addr + PIO_INT_STAT_OFFSET);
	reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
	reg_val |=(1<<IRQ_EINT21);
	writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);
	// Re-enable IRQ so that we can handle the next ZForce event
	//	enable_irq(priv->irq);
	while ((reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET))&(1<<IRQ_EINT21)) {
		//      printk("============== Clear EINT21 ================\n");
		writel(reg_val&(1<<(IRQ_EINT21)),gpio_addr + PIO_INT_STAT_OFFSET);
	}
	enable_irq(SW_INT_IRQNO_PIO);
	return;
}

static void zforce_ts_handle_deactivate(u_int8_t *data, int len) {

	printk("ZForce deactivation response, status=%d\n", data[0]);
}

static void zforce_ts_handle_activate(u_int8_t *data, int len) {

	printk("ZForce activation response, status=%d\n", data[0]);
}

static void zforce_ts_handle_configure(u_int8_t *data, int len) {

	printk("ZForce configuration response, status=%d\n", data[0]);
}

static void zforce_ts_handle_setres(u_int8_t *data, int len) {

	printk("ZForce resolution setting response, status=%d\n", data[0]);
}


// Handle touch coordinate events form the ZF
static void zforce_ts_handle_touchdata(struct zforce_ts_priv *priv, u_int8_t *payload, int len) 
{
	// Be more verbose for the very first touch coordinate we get
	int more_verbose = priv->startup_state & ZF_FIRSTTOUCH_RESPONDED ? 0 : 1;
	// The number of touch coordinates reported by the ZForce
	int coordinate_count = payload[0];
	// Used to automatically determine protocol version
	int protocol_variant = 0;
	// Index within the coordinate sub-data blocks
	int coord_index;
	int point = 0;
	int x ,y;
	int x1;
	int y1;
	int x2;
	int y2;
	int status;
	int i;

	// This is a QaD trick to determine which of the two protocol flavors we got (one uses 5 bytes
	//  per coordinate, the other uses 7 bytes per coordinate). The "protocol_variant" is set to the
	//  coordinate block length
	if (len <= 2){
		ctp_wakeup();
		zforce_ts_send_activate(priv);	
	
	} else if (len == coordinate_count * 5 + 1) {
		if (more_verbose) {
			printk("Using 5 bytes per coordinate protocol\n");
		}
		// Older ZForce with 5 bytes per coordinate
		protocol_variant = 5;
	} else if (len == coordinate_count * 9 + 1) {
		if (more_verbose) {
			//printk("Using 5 bytes per coordinate protocol\n");
		}
		// Newer ZForce with 7 bytes per coordinate
		protocol_variant = 9;
	}
	printk("\n+++++++++++++++++++++++++++++++++++++++Touch_X_Y_TS+++++++++++++++++++++++++++\n");
	// Did we figure out the protocol ?
	if (!protocol_variant) {
		// Not the expected packet length
		dev_err(&priv->client->dev,
				"Could not match ZForce block length to any protocol\n");
		return;
	}

	for (i = 1; i < len; i++) 
	{ // Scan the buffer (skip the coordinate count)
		int data = payload[i];
		//dual touch event
		// The index within the multi-touch coordinate we are currently parsing
		
		if (coordinate_count == 1) //single touch event
		{
				coord_index = (i - 1) % (protocol_variant );
				if (more_verbose && !coord_index) 
				{
					//printk("Parsing ZForce coordinate %d\n", i / protocol_variant);
				}
				switch (coord_index) 
				{
					case 0: // X LSB
						x1 = data;
						break;

					case 1: // X MSB
						x1 |= (data << 8);
						break;

					case 2: // Y LSB
						y1 = data;
						break;

					case 3: // Y MSB
						y1 |= (data << 8);
						break;

					case 4: // Status
						status = data;
						if (more_verbose) {
							//	printk("Status=0x0%x x=%d y=%d\n", status, x, y);
						}
						//printk("Status1=0x0%x x1=%d y1=%d\n", status, x1, y1);
						x1 = (ZF_MAX_X - x1) ;
						y1 = (ZF_MAX_Y - y1) ;
						//printk("__Status1=0x0%x __x1=%d  __y1=%d\n", status, x1, y1);
						// The status bits differ between the two "protocol variants"
						switch (status) {
							case 0x04: // Touch down
								input_report_abs(priv->input, ABS_MT_POSITION_X, x1);
								input_report_abs(priv->input, ABS_MT_POSITION_Y, y1);
								input_report_abs(priv->input, ABS_MT_TOUCH_MAJOR, 1);
								input_mt_sync(priv->input);
								break;

							case 0x05: // Move
								input_report_abs(priv->input, ABS_MT_POSITION_X, x1);
								input_report_abs(priv->input, ABS_MT_POSITION_Y, y1);
								input_mt_sync(priv->input);
								break;

							case 0x06: // Up
								input_report_abs(priv->input, ABS_MT_TOUCH_MAJOR, 0);
								input_mt_sync(priv->input);
								break;
						}
						break;

					case 5: // Reserved byte
					case 6: // Probability, disregarded for the moment
					case 7:
					case 8:
						break;

				}

		} else {
			coord_index = (i - 1) % (protocol_variant );
			if (more_verbose && !coord_index) 
			{
				//printk("Parsing ZForce coordinate %d\n", i / protocol_variant);
			}
			switch (coord_index) 
			{
				case 0: // X LSB
					point ++;
					x = data;
					break;

				case 1: // X MSB
					x |= (data << 8);
					break;

				case 2: // Y LSB
					y = data;
					break;

				case 3: // Y MSB
					y |= (data << 8);
					break;

				case 4: // Status
					status = data;
					break;

				case 5: // Reserved byte
				case 6: // Probability, disregarded for the moment
				case 7:
				case 8:
					break;
			}

			if (more_verbose) 
			{
				//	printk("Status=0x0%x x=%d y=%d\n", status, x, y);
			}
			//printk("Status1=0x0%x x1=%d y1=%d\n", status, x1, y1);
			if (point == 1)
			{
				x1 = x;
				y1 = y;
				x1 = (ZF_MAX_X - x1) ;
				y1 = (ZF_MAX_Y - y1) ;
			}else if (point == 2)
			{
				x2 = x;
				y2 = y;
				x2 = (ZF_MAX_X - x2) ;
				y2 = (ZF_MAX_Y - y2) ;
			}
			//printk("__Status1=0x0%x __x1=%d  __y1=%d\n", status, x1, y1);
			//printk("__Status1=0x0%x __x2=%d  __y2=%d\n", status, x2, y2);
			// The status bits differ between the two "protocol variants"

			switch (status) 
			{
				case 0x08: // Touch down
					//input_report_key(priv->input, ABS_MT_TRACKING_ID, 1); 
					input_report_abs(priv->input, ABS_MT_POSITION_X, x1);
					input_report_abs(priv->input, ABS_MT_POSITION_Y, y1);
					input_report_abs(priv->input, ABS_MT_TOUCH_MAJOR, 1);
					input_mt_sync(priv->input);

					//input_report_key(priv->input, ABS_MT_TRACKING_ID, 1); 
					input_report_abs(priv->input, ABS_MT_POSITION_X, x2);
					input_report_abs(priv->input, ABS_MT_POSITION_Y, y2);
					input_report_abs(priv->input, ABS_MT_TOUCH_MAJOR, 1);
					input_mt_sync(priv->input);
					break;

				case 0x09: // Move
					//input_report_key(priv->input, ABS_MT_TRACKING_ID, 1); 
					input_report_abs(priv->input, ABS_MT_POSITION_X, x1);
					input_report_abs(priv->input, ABS_MT_POSITION_Y, y1);
					input_mt_sync(priv->input);

					//input_report_key(priv->input, ABS_MT_TRACKING_ID, 1); 
					input_report_abs(priv->input, ABS_MT_POSITION_X, x2);
					input_report_abs(priv->input, ABS_MT_POSITION_Y, y2);
					input_mt_sync(priv->input);
					break;

				case 0x0a: // Up
					input_report_abs(priv->input, ABS_MT_TOUCH_MAJOR, 0);
					input_mt_sync(priv->input);
					break;
				}
			}
			
		}

		if (!(i % protocol_variant)) {
			// This is the last byte of the coordinate, so we can fire off the MT sync here
			input_mt_sync(priv->input);
			if (more_verbose) {
				printk("Syncing touch coordinate\n");
			}
		}
		// Final sync
		input_sync(priv->input);
		if (more_verbose) {
			printk("Syncing multi touch event\n");
		}
}

static void zforce_ts_handle_version(u_int8_t *data, int len) {

	u_int16_t major = data[0] + (data[1] << 8);
	u_int16_t minor = data[2] + (data[3] << 8);
	u_int16_t build = data[4] + (data[5] << 8);
	u_int16_t revision = data[6] + (data[7] << 8);

	printk("ZForce version %d.%d, build %d, rev %d\n", major, minor, build,
			revision);
}

static int zforce_ts_send_deactivate(struct zforce_ts_priv *priv) {

	u_int8_t data[] = { 0xee, 0x01,  ZF_DEACTIVATE };

	printk("send:  Deactivating zForce command:  0xee, 0x01, 0x00 \n");

	return i2c_master_send(priv->client, data, sizeof(data));
}

static int zforce_ts_send_activate(struct zforce_ts_priv *priv) {

	u_int8_t data[] = { 0xee, 0x01, ZF_ACTIVATE };

	printk("send: Activating zForce: 0xee, 0x01, 0x01 \n");

	return i2c_master_send(priv->client, data, sizeof(data));
}

static int zforce_ts_send_configure(struct zforce_ts_priv *priv, int dual_touch) {

	u_int8_t data[] = {0xee, 0x05, ZF_CONFIGURE, 0, 0, 0, 0 };
	printk("send: 0xee, 0x05, 0x03, 0,0,0 0");
	printk("Configuring zForce, using %s touch\n",
			dual_touch ? "dual" : "single");

	data[3] = dual_touch ? 0x01 : 0x00;

	return i2c_master_send(priv->client, data, sizeof(data));
}


//add by Ethan
static int zforce_ts_send_areasize(struct zforce_ts_priv *priv)
{
	u_int8_t data[] = {0xee, 0x05, ZF_AREA_SIZE,  0x01, 0xff, 0x01, 0xff};

	printk("send: touch Area zForce:  0xee, 0x05, 0x09, 0x01, 0xff, 0x01, 0xff \n");

	return i2c_master_send(priv->client, data, sizeof(data));

}

static int zforce_ts_send_calibration(struct zforce_ts_priv *priv)
{

	u_int8_t data[] = {0xee, 0x01,ZF_CALIBRATION };

	printk("send: calibraion of zForce: 0xee, 0x01, 0x1a  \n");

	return i2c_master_send(priv->client, data, sizeof(data));
}


static void zforce_ts_handle_toucharea(u_int8_t *data, int len)
{
	printk("==zforce_ts_handle_toucharea==\n ");
}


static void zforce_ts_handle_calibration(u_int8_t *data, int len)
{
	printk("==zforce_ts_handle_calibration ==\n");
}
//end add

static int zforce_ts_send_setres(struct zforce_ts_priv *priv, int width,
		int height) {

	u_int8_t data[] = {0xee, 0x05,  ZF_SET_RESOLUTION, 0, 0, 0, 0 };

	printk("Setting ZForce resolution, width=%d, height=%d\n", width, height);

	data[3] = width & 0xff;
	data[4] = width >> 8;
	data[5] = height & 0xff;
	data[6] = height >> 8;
	
	printk("send: resolution command: 0xee, 0x05, 0x02,%x, %x, %x, %x\n ",width & 0xff, width >>8 , height & 0xff, height >> 8);

	return i2c_master_send(priv->client, data, sizeof(data));
}

static int zforce_ts_send_touchdata_req(struct zforce_ts_priv *priv) {
	u_int8_t data[] = {0xee, 0x01,  ZF_REQ_COORDINATES };
	u_int8_t tmp_buf[10] = {0};
	int i = 0,j = 0;
	if (!(priv->startup_state & ZF_FIRSTTOUCH_RESPONDED)) {

		printk("Requesting touch coordinates\n");
	}
	
	printk("send: TouchDataRequest: 0xee, 0x01, 0x04\n");
		
	 i2c_master_send(priv->client, data, sizeof(data));
	for(;i < 10; i++){
		 i2c_master_recv(priv->client, tmp_buf, 10);
		for(;j < 10; j++){
			printk("%2x \t", tmp_buf[j]);
		}
		printk("\n");
	}
}

static int zforce_ts_send_version_req(struct zforce_ts_priv *priv) {

	u_int8_t data[] = { 0xee, 0x01, ZF_REQ_VERSION };

	printk("send: Getting ZForce version: 0xee , 0x01, 0x0a\n");

	return i2c_master_send(priv->client, data, sizeof(data));
}

static irqreturn_t zforce_ts_isr(int irq, void *dev_id) {
	struct zforce_ts_priv *priv = dev_id;


	int reg_val, tmp;
	disable_irq_nosync(SW_INT_IRQNO_PIO);
	printk("==interrupt generate==\n");
	//	clear the IRQ_EINT21 interrupt pending
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(IRQ_EINT21)))
	{
		tmp = reg_val;
		reg_val = readl(gpio_addr + PIO_INT_CTRL_OFFSET);
		reg_val |=(0<<IRQ_EINT21);
		writel(reg_val,gpio_addr + PIO_INT_CTRL_OFFSET);

		//clean irq
		reg_val = tmp;
		writel(reg_val&(1<<(IRQ_EINT21)),gpio_addr + PIO_INT_STAT_OFFSET);

		schedule_work(&priv->work);

	}    
	else 
	{    
	#ifdef PRINT_INT_INFO
		printk("Other Interrupt\n");
	#endif
		return IRQ_NONE;
	}		

	return IRQ_HANDLED;
}

static int zforce_ts_open(struct input_dev *dev) {

#if 0 
	struct zforce_ts_priv *priv = input_get_drvdata(dev);
	struct i2c_client *client = priv->client;

	// Fire off a version request to get the state machinery up and running, the rest
	// is done by the ISR handling routine
	if (zforce_ts_send_version_req(priv) < 0) {

		dev_err(&client->dev,
				"Unable to request version from ZForce touchscreen.\n");
		return -ENXIO;
	}
#endif
	return 0;
}

static void zforce_ts_close(struct input_dev *dev) {
	struct zforce_ts_priv *priv = input_get_drvdata(dev);
	
	disable_irq(priv->irq);
	printk("=======================================zforce_deactivate_TSS========================================");
	// Cancel any pending bottom half work (wait for it to finish)
/*
	if (cancel_delayed_work_sync(&priv->work)) {

		// Re-enable IRQ as it might have been disabled by the "top half" ISR
		enable_irq(priv->irq);
	}
*/
	//TOOD: Maybe this should be done before cancelling work...
	// Deactivate the touch screen
	zforce_ts_send_deactivate(priv);
//	enable_irq(priv->irq);
}

static int
zforce_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct zforce_ts_priv *priv = NULL;
	struct input_dev *input = NULL;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int error;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate driver data\n");
		error = -ENOMEM;
		goto err0;
	}

	dev_set_drvdata(&client->dev, priv);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "Failed to allocate input device.\n");
		error = -ENOMEM;
		goto err1;
	}

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

// add by Ethan
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN, input->evbit);
	
	set_bit(ABS_X, input->absbit);
	set_bit(ABS_Y, input->absbit);
	set_bit(ABS_PRESSURE, input->absbit);
	set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, PRESS_MAX, 0 , 0);
	
    set_bit(ABS_MT_POSITION_X, input->absbit);
    set_bit(ABS_MT_POSITION_Y, input->absbit);
    set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);

	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,	0, 15, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_X, 0, ZF_MAX_X, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ZF_MAX_Y, 0, 0);
//end add

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->open = zforce_ts_open;

	input_set_drvdata(input, priv);

	priv->client = client;
	priv->input = input;

/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	pr_info("==register_early_suspend =\n");
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	priv->early_suspend.suspend = zforce_ts_suspend;
	priv->early_suspend.resume	= zforce_ts_resume;
	register_early_suspend(&priv->early_suspend);
#endif*/

//modify
	//INIT_DELAYED_WORK(&priv->work, zforce_ts_handle_data);
	INIT_WORK(&priv->work, zforce_ts_handle_data);
	priv->irq = client->irq;

	error = input_register_device(input);
	if (error)
		goto err1;

	//TODO: Even though the ISR is setup here, the IRQ line must be setup in the init
	//  or in the board support routine. For OMAP, the OMAP_GPIO_IRQ can be used to convert
	//  pin number into IRQ
	// See gpio_request, gpio_direction_input, gpio_export
	// See http://old.nabble.com/GPIO-programming-and-Interrupts-td24324179.html

	// See drivers/input/touchscreen/mainstone-wm97xx.c


//	schedule_delayed_work(&priv->work, HZ / 20);

	error = request_irq(SW_INT_IRQNO_PIO, zforce_ts_isr, IRQF_TRIGGER_LOW | IRQF_SHARED,
		client->name, priv);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err2;
	}
	ctp_set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_MODE);

	
//add by Ethan

	set_bit(INPUT_PROP_DIRECT, input->propbit); //cancle mouse icon
	
//add end

    	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)){
		error = PTR_ERR(i2c_dev);
		return error;
	}
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,client->adapter->nr), NULL, "aw_i2c_ts%d", client->adapter->nr);
	if (IS_ERR(dev))	{
			error = PTR_ERR(dev);
			return error;
	}

	device_init_wakeup(&client->dev, 1);
	return 0;

err2: input_unregister_device(input);
	  input = NULL; /* so we dont try to free it below */
err1: input_free_device(input);
	  kfree(priv);
err0: dev_set_drvdata(&client->dev, NULL);
	  return error;

}

static int __devexit zforce_ts_remove(struct i2c_client *client)
{

	struct zforce_ts_data *zforce_ts = i2c_get_clientdata(client);

	pr_info("==zforce_ts_remove=\n");
	free_irq(SW_INT_IRQNO_PIO, zforce_ts);
/*#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&zforce_ts->early_suspend);
#endif*/
	input_unregister_device(zforce_ts->input_dev);
	input_free_device(zforce_ts->input_dev);
	cancel_work_sync(&zforce_ts->pen_event_work);
	destroy_workqueue(zforce_ts->ts_workqueue);
	kfree(zforce_ts);

	i2c_set_clientdata(client, NULL);
	ctp_ops.free_platform_resource();

	return 0;

}

static const struct i2c_device_id zforce_ts_id[] = {
	{ CTP_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, zforce_ts_id);

static struct i2c_driver zforce_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= zforce_ts_probe,
	.remove		= __devexit_p(zforce_ts_remove),
	.id_table	= zforce_ts_id,
	.driver	= {
		.name	= CTP_NAME,
		.owner	= THIS_MODULE,
	},
	.address_list	= u_i2c_addr.normal_i2c,
};

static int aw_open(struct inode *inode, struct file *file)
{
	int subminor;
	int ret = 0;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;

	pr_info("====%s======.\n", __func__);

	#ifdef AW_DEBUG
	        pr_info("enter aw_open function\n");
	#endif

	subminor = iminor(inode);
	#ifdef AW_DEBUG
	      pr_info("subminor=%d\n",subminor);
	#endif

	//lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(2);
	if (!i2c_dev)	{
		pr_info("error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter)	{
		return -ENODEV;
	}

	client = kzalloc(sizeof(*client), GFP_KERNEL);

	if (!client)	{
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "pctp_i2c_ts%d", adapter->nr);
	client->driver = &zforce_ts_driver;
	client->adapter = adapter;
	file->private_data = client;

	return 0;
}

static long aw_ioctl(struct file *file, unsigned int cmd,unsigned long arg )
{
	//struct i2c_client *client = (struct i2c_client *) file->private_data;

	pr_info("====%s====.\n",__func__);

	#ifdef AW_DEBUG
	       pr_info("line :%d,cmd = %d,arg = %d.\n",__LINE__,cmd,arg);
	#endif

	switch (cmd) {
		case UPGRADE:
		pr_info("==UPGRADE_WORK=\n");
		fts_ctpm_fw_upgrade_with_i_file();
		// calibrate();

		break;

		default:
		break;

	}

	return 0;
}

static int aw_release (struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;
	#ifdef AW_DEBUG
	    pr_info("enter aw_release function.\n");
	#endif

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;
	return 0;
}

static const struct file_operations aw_i2c_ts_fops ={
	.owner = THIS_MODULE,
	//.read = aw_read,
	//.write = aw_write,
	.open = aw_open,
	.unlocked_ioctl = aw_ioctl,
	.release = aw_release,
};

static int __init zforce_ts_init(void)
{
	int ret = -1;
	int err = -1;

	pr_info("===========================%s=====================\n", __func__);

	if (ctp_ops.fetch_sysconfig_para)
	{
		if(ctp_ops.fetch_sysconfig_para()){
			pr_info("%s: err.\n", __func__);
			return -1;
		}
	}
	pr_info("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	err = ctp_ops.init_platform_resource();
	if(0 != err){
	    pr_info("%s:ctp_ops.init_platform_resource err. \n", __func__);
	}

	//reset
	ctp_ops.ts_reset();
	//wakeup
	ctp_ops.ts_wakeup();

	zforce_ts_driver.detect = ctp_ops.ts_detect;

	ret= register_chrdev(I2C_MAJOR,"aw_i2c_ts",&aw_i2c_ts_fops );
	if(ret) {
		pr_info(KERN_ERR "%s:register chrdev failed\n",__FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE,"aw_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	ret = i2c_add_driver(&zforce_ts_driver);
	return ret;
}

static void __exit zforce_ts_exit(void)
{
	pr_info("==zforce_ts_exit==\n");
	i2c_del_driver(&zforce_ts_driver);
}

late_initcall(zforce_ts_init);
module_exit(zforce_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech zforce TouchScreen driver");
MODULE_LICENSE("GPL");

