/*
 * platform_ftxxxx.c: ftxxxx platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/intel_scu_flis.h>

#include "platform_ftxxxx.h"

#ifdef CONFIG_ARES8
	/* Values from Ares8 2016/08 */
	#define FTXXXX_I2C_BUS 4
	#define FTXXXX_I2C_ADDR 0x38
#else
	/* Values from zenfone2 driver */
	#define FTXXXX_I2C_BUS 7
	#define FTXXXX_I2C_ADDR (0x70 >> 1)
#endif

static struct ftxxxx_platform_data ftxxxx_pdata = {
	.gpio_irq = FTXXXX_INT_PIN,
	.gpio_reset = FTXXXX_RESET_PIN,
	.screen_max_x = TOUCH_MAX_X,
	.screen_max_y = TOUCH_MAX_Y,
};

static struct i2c_board_info bus4_i2c_devices[] = {
	{
		I2C_BOARD_INFO(FTXXXX_NAME, FTXXXX_I2C_ADDR),
		.platform_data = &ftxxxx_pdata,
	},
};
//static struct spi_board_info spi_devices[] = {};

static int __init ftxxxx_init(void)
{
	int ret;

	ret = i2c_register_board_info(FTXXXX_I2C_BUS, &bus4_i2c_devices, ARRAY_SIZE(bus4_i2c_devices));
	printk("[%s]i2c_register_board_info : %d \n",__func__ ,ret);

	return ret;
}

arch_initcall(ftxxxx_init);


