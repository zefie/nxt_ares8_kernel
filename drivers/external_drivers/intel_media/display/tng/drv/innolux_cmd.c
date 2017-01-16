/*
 * Copyright © 2015 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors: Sophia Gong <sophia.gong@intel.com>
 *
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_esd.h"
#include "mdfld_dsi_dbi_dsr.h"
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/lnw_gpio.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>

#include "displays/innolux_cmd.h"
#include "innolux_init.h"

static int select_init_code;

static bool reset_enable = false;

static int __init parse_panel_init_code(char *arg)
{
	sscanf(arg, "%d", &select_init_code);

	return 1;
}
early_param("panel_init_code", parse_panel_init_code);

#ifdef INNOLUX_DEBUG
static int readback_initcode(struct mdfld_dsi_config *);
#endif

static int mipi_reset_gpio;
static int bias_en_gpio;

typedef struct {
	struct dentry *dir;
/* atomic ops */
	struct dentry *addr_set;
	struct dentry *send_lp;
	struct dentry *send_hs;
	struct dentry *read_lp;
	struct dentry *read_hs;
	unsigned int addr;
} dbgfs_t;


static dbgfs_t dbgfs;
static struct mdfld_dsi_config *dbgfs_dsi_config;

static
int innolux_cmd_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	msleep(120);

	/* set LCD panel CMD1 */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			0xff, 0x10, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: 0xff cmd\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	/* set DC VCOM */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			0xb3, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: 0xb3 cmd\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	/* set pixel format */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			0xc0, 0x01, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: 0xc0 cmd\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	/* set panel command mode */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			0xbb, 0x10, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: 0xbb cmd\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	/* set LCD panel CMD1 */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			0xff, 0x10, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: 0xfb cmd\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	/* turn off MTP reload */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			0xfb, 0x01, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: 0xfb cmd\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

#ifdef INNOLUX_DEBUG
	readback_initcode(dsi_config);
#endif
	return 0;

ic_init_err:
	err = -EIO;
	return err;
}

static
int innolux_cmd_drv_ic_fullinit(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	int i;
	u8 *cmd, *arg;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	msleep(120);

	cmd = (u8 *)innolux_init_code;
	arg = cmd + 1;

	for (i = 0; i < ARRAY_SIZE(innolux_init_code); i++) {
		err = mdfld_dsi_send_mcs_short_lp(sender,
				*cmd, *arg, 1,
				MDFLD_DSI_SEND_PACKAGE);
		cmd += 2;
		arg += 2;
		if (err) {
			DRM_ERROR("%s: %d: 0xff cmd\n",
			__func__, __LINE__);
			goto ic_init_err2;
		}

#ifdef INNOLUX_DEBUG
		DRM_INFO("panel init: %x=%x\n", *cmd, *arg);
#endif
	}

ic_init_err2:
	if (err)
		err = -EIO;
#ifdef INNOLUX_DEBUG
	readback_initcode(dsi_config);
#endif
	return err;
}

static
void innolux_cmd_controller_init(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
				&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 1;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;

	/* DSI PLL 400 MHz, set it to 0 for 800 MHz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->turn_around_timeout = 0x1a;
	hw_ctx->high_low_switch_count = 0xe;
	hw_ctx->clk_lane_switch_time_cnt = 0xe0008;
	hw_ctx->lp_byteclk = 0x2;
	hw_ctx->dphy_param = 0x120a2b0c;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0xf0;
	hw_ctx->dbi_bw_ctrl = 1390;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	hw_ctx->dsi_func_prg = ((DBI_DATA_WIDTH_OPT2 << 13) |
				dsi_config->lane_count);
		hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE |
			BANDGAP_CHICKEN_BIT |
			TE_TRIGGER_GPIO_PIN;
	hw_ctx->panel_on = true;

/*re-enable reset fuction*/
	reset_enable = true;

        /*
         * local copy of dsi_config needed by debugfs to use:
         * - mdfld_dsi_send_*
         * - mdfld_dsi_read_*
         */
	if (dbgfs_dsi_config == NULL)
		dbgfs_dsi_config = dsi_config;
}

static
int innolux_cmd_panel_connection_detect(
	struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		DRM_INFO("%s: do NOT support dual panel\n",
		__func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int innolux_cmd_power_on(
	struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	usleep_range(300000, 301000);

	/* Exit sleep mode */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			exit_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Exit Sleep Mode\n",
		__func__, __LINE__);
		goto power_err;
	}

	msleep(120);

	/* turn on display */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_display_on, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
		goto power_err;
	}

	msleep(60);

	/* set backlight on */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_display, 0x2c, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Backlight On\n",
		__func__, __LINE__);
		goto power_err;
	}

	/* set backlight brightness */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_display_brightness, 0xff, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Backlight Brightness\n",
		__func__, __LINE__);
		goto power_err;
	}

	/* set CABC/IE disable */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_cabc, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set CABC/IE Disable\n",
		__func__, __LINE__);
		goto power_err;
	}

	/* set tearing effect on */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_tear_on, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Tear On\n",
		__func__, __LINE__);
		goto power_err;
	}

	usleep_range(20000, 20100);
	return 0;

power_err:
	return err;
}

static int innolux_cmd_power_off(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* set backlight off */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_ctrl_display, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Backlight Off\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	/* set backlight brightness */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			write_display_brightness, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Backlight Brightness\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	usleep_range(1000, 1100);

	/* set display off */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			set_display_off, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display Off\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	/* set sleep-in */
	err = mdfld_dsi_send_mcs_short_lp(sender,
			enter_sleep_mode, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Sleep-in\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	/* assert panel reset : delay > 85 ms */
	usleep_range(85000, 85100);
	gpio_set_value(mipi_reset_gpio, 0);

	if (bias_en_gpio)
		gpio_set_value(bias_en_gpio, 0);

	return 0;

power_off_err:
	err = -EIO;
	return err;
}

static
int innolux_cmd_set_brightness(
		struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 duty_val = 0;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (0xFF * level) / 255;
	mdfld_dsi_send_mcs_short_hs(sender,
			write_display_brightness, duty_val, 1,
			MDFLD_DSI_SEND_PACKAGE);
	return 0;
}

static
int innolux_cmd_panel_reset(
		struct mdfld_dsi_config *dsi_config)
{
	u8 value;

/*we don't want to reset the display. During the boot it is already on*/
	if (reset_enable == false)
		return 0;

	PSB_DEBUG_ENTRY("\n");

	gpio_direction_output(bias_en_gpio, 1);
	gpio_direction_output(mipi_reset_gpio, 0);

	gpio_set_value(bias_en_gpio, 1);
	gpio_set_value(mipi_reset_gpio, 0);

	/* turn on VSWITCH */
	intel_scu_ipc_ioread8(0xAF, &value);
	intel_scu_ipc_iowrite8(0xAF, value | 0x2);

	usleep_range(10000, 12000);

	gpio_set_value(mipi_reset_gpio, 1);

	usleep_range(10000, 12000);

	return 0;
}

static
int innolux_cmd_exit_deep_standby(
		struct mdfld_dsi_config *dsi_config)
{
	PSB_DEBUG_ENTRY("\n");

	if (bias_en_gpio)
		gpio_set_value(bias_en_gpio, 1);

	gpio_direction_output(mipi_reset_gpio, 0);

	gpio_set_value(mipi_reset_gpio, 0);
	usleep_range(3000, 3100);

	gpio_set_value(mipi_reset_gpio, 1);
	usleep_range(10000, 12000);

	return 0;
}

static
struct drm_display_mode *innolux_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 360;
	mode->hsync_start = 392;
	mode->hsync_end = 393;
	mode->htotal = 425;

	mode->vdisplay = 360;
	mode->vsync_start = 370;
	mode->vsync_end = 371;
	mode->vtotal = 377;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void innolux_cmd_get_panel_info(int pipe,
		struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = 40;
		pi->height_mm = 40;
	}
}

/* atomic operations from debugfs */
enum dbgfs_type { ADDR, HIGH_SPEED, LOW_POWER };
/* reading operations */
static ssize_t dbgfs_read(char __user *, size_t , loff_t *, enum dbgfs_type);
static ssize_t dbgfs_addr_read(struct file *, char __user *, size_t , loff_t *);
static ssize_t dbgfs_read_hs_read(struct file *, char __user *, size_t , loff_t *);
static ssize_t dbgfs_read_lp_read(struct file *, char __user *, size_t , loff_t *);
/* writting operations */
static int dbgfs_write(const char __user *buff, size_t , enum dbgfs_type);
static ssize_t dbgfs_addr_write(struct file *, const char __user *, size_t , loff_t *);
static ssize_t dbgfs_send_lp_write(struct file *, const char __user *, size_t , loff_t *);
static ssize_t dbgfs_send_hs_write(struct file *, const char __user *, size_t , loff_t *);

/* ops for atomic operations */
static const struct file_operations dbgfs_addr_ops = {
	.open		= nonseekable_open,
	.read		= dbgfs_addr_read,
	.write		= dbgfs_addr_write,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_send_lp_ops = {
	.open		= nonseekable_open,
	.write		= dbgfs_send_lp_write,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_send_hs_ops = {
	.open		= nonseekable_open,
	.write		= dbgfs_send_hs_write,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_read_lp_ops = {
	.open		= nonseekable_open,
	.read		= dbgfs_read_lp_read,
	.llseek		= no_llseek,
};
static const struct file_operations dbgfs_read_hs_ops = {
	.open		= nonseekable_open,
	.read		= dbgfs_read_hs_read,
	.llseek		= no_llseek,
};

void innolux_cmd_init(struct drm_device *dev,
		struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	bias_en_gpio = get_gpio_by_name("disp0_bias_en");
	if (bias_en_gpio <= 0)
		bias_en_gpio = 189;
	gpio_request(bias_en_gpio, "innolux_display");

	mipi_reset_gpio = get_gpio_by_name("disp0_rst");
	if (mipi_reset_gpio <= 0)
		mipi_reset_gpio = 190;
	gpio_request(mipi_reset_gpio, "innolux_display");

	lnw_gpio_set_alt(68, 1); /* Force TE as muxmode1:
				this should not be necessary as already done in IFWI */

	PSB_DEBUG_ENTRY("\n");
	p_funcs->reset = innolux_cmd_panel_reset;
	p_funcs->power_on = innolux_cmd_power_on;
	p_funcs->power_off = innolux_cmd_power_off;
	p_funcs->drv_ic_init = (select_init_code) ? innolux_cmd_drv_ic_fullinit : innolux_cmd_drv_ic_init;
	p_funcs->get_config_mode = innolux_cmd_get_config_mode;
	p_funcs->get_panel_info = innolux_cmd_get_panel_info;
	p_funcs->dsi_controller_init = innolux_cmd_controller_init;
	p_funcs->detect = innolux_cmd_panel_connection_detect;
	p_funcs->set_brightness = innolux_cmd_set_brightness;
	p_funcs->exit_deep_standby = innolux_cmd_exit_deep_standby;

	/* debugfs */
	dbgfs_dsi_config = NULL;

	dbgfs.dir = debugfs_create_dir("innolux", NULL);
	if (dbgfs.dir == NULL) {
		DRM_ERROR("%s-%d: cannot create debugfs directory\n", __func__, __LINE__);
		return;
	}
	/* atomic ops */
	dbgfs.addr_set = debugfs_create_file("addr",
				S_IFREG | S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
				dbgfs.dir, NULL,
				&dbgfs_addr_ops);
	if (dbgfs.addr_set == NULL)
		DRM_ERROR("%s-%d: cannot create debugfs entry addr_set\n", __func__, __LINE__);

	dbgfs.send_lp = debugfs_create_file("send_lp",
				S_IFREG | S_IWUSR | S_IWGRP,
				dbgfs.dir, NULL,
				&dbgfs_send_lp_ops);
	if (dbgfs.send_lp == NULL)
		DRM_ERROR("%s-%d: cannot create debugfs entry send_lp\n", __func__, __LINE__);

	dbgfs.send_hs = debugfs_create_file("send_hs",
				S_IFREG | S_IWUSR | S_IWGRP,
				dbgfs.dir, NULL,
				&dbgfs_send_hs_ops);
	if (dbgfs.send_hs == NULL)
		DRM_ERROR("%s-%d: cannot create debugfs entry send_hs\n", __func__, __LINE__);

	dbgfs.read_lp = debugfs_create_file("read_lp",
				S_IFREG | S_IRUSR | S_IRGRP,
				dbgfs.dir, NULL,
				&dbgfs_read_lp_ops);
	if (dbgfs.read_lp == NULL)
		DRM_ERROR("%s-%d: cannot create debugfs entry read_lp\n", __func__, __LINE__);

	dbgfs.read_hs = debugfs_create_file("read_hs",
				S_IFREG | S_IRUSR | S_IRGRP,
				dbgfs.dir, NULL,
				&dbgfs_read_hs_ops);
	if (dbgfs.read_hs == NULL)
		DRM_ERROR("%s-%d: cannot create debugfs entry read_hs\n", __func__, __LINE__);
}

/* atomic operations */
static ssize_t dbgfs_read(char __user *buff, size_t count, loff_t *ppos, enum dbgfs_type type)
{
	char *str;
	u8 data = 0;
	ssize_t len = 0;
	u32 power_island = 0;
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dbgfs_dsi_config);
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	/* setting display and MIPI bus in correct state for reading */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		power_island = pipe_to_island(dbgfs_dsi_config->pipe);

		if (power_island & (OSPM_DISPLAY_A | OSPM_DISPLAY_C))
			power_island |= OSPM_DISPLAY_MIO;

		if (!power_island_get(power_island))
			return -EIO;

		mdfld_dsi_dsr_forbid(dbgfs_dsi_config);
	}

	str = kzalloc(count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	switch (type) {
	case ADDR:
		len = sprintf(str, "addr = 0x%x\n", (u8)dbgfs.addr);
		break;
	case HIGH_SPEED:
		mdfld_dsi_read_mcs_hs(sender, (u8)dbgfs.addr, &data, 1);
		break;
	case LOW_POWER:
		mdfld_dsi_read_mcs_lp(sender, (u8)dbgfs.addr, &data, 1);
		break;
	}

	/* releasing display and MIPI bus */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		len = sprintf(str, "addr = 0x%x, value = 0x%x\n", (u8)dbgfs.addr, data);
		mdfld_dsi_dsr_allow(dbgfs_dsi_config);
		power_island_put(power_island);
	}

	if (len < 0)
		DRM_ERROR("Can't read data\n");
	else
		len = simple_read_from_buffer(buff, count, ppos, str, len);

	kfree(str);

	return len;
}

static ssize_t dbgfs_addr_read(struct file *file, char __user *buff,
				size_t count, loff_t *ppos)
{
	ssize_t len = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	len = dbgfs_read(buff, count, ppos, ADDR);

	return len;
}

static ssize_t dbgfs_read_hs_read(struct file *file, char __user *buff,
				   size_t count, loff_t *ppos)
{
	ssize_t len;

	if (*ppos < 0 || !count)
		return -EINVAL;

	len = dbgfs_read(buff, count, ppos, HIGH_SPEED);

	return len;
}

static ssize_t dbgfs_read_lp_read(struct file *file, char __user *buff,
				   size_t count, loff_t *ppos)
{
	ssize_t len;

	if (*ppos < 0 || !count)
		return -EINVAL;

	len = dbgfs_read(buff, count, ppos, LOW_POWER);

	return len;
}

static int dbgfs_write(const char __user *buff, size_t count, enum dbgfs_type type)
{
	int err = 0, i = 0, ret = 0;
	unsigned int arg = 0;
	u32 power_island = 0;
	char *start, *str;
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dbgfs_dsi_config);
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	str = kzalloc(count, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	if (copy_from_user(str, buff, count)) {
		ret = -EFAULT;
		goto exit_dbgfs_write;
	}

	start = str;

	while (*start == ' ')
		start++;

	/* strip ending whitespace */
	for (i = count - 1; i > 0 && isspace(str[i]); i--)
		str[i] = 0;

	/* setting display and MIPI bus in correct state for writting */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		power_island = pipe_to_island(dbgfs_dsi_config->pipe);

		if (power_island & (OSPM_DISPLAY_A | OSPM_DISPLAY_C))
			power_island |= OSPM_DISPLAY_MIO;

		if (!power_island_get(power_island)) {
			ret = -EIO;
			goto exit_dbgfs_write;
		}

		mdfld_dsi_dsr_forbid_locked(dbgfs_dsi_config);

		if (kstrtouint(start, 16, &arg)) {
			ret = -EINVAL;
			goto exit_dbgfs_write;
		}
	}

	switch (type) {
	case ADDR:
		if (kstrtouint(start, 16, &(dbgfs.addr)))
			ret = -EINVAL;
		break;
	case HIGH_SPEED:
		err = mdfld_dsi_send_mcs_short_hs(sender,
					(u8)dbgfs.addr, (u8)arg, 1,
					MDFLD_DSI_SEND_PACKAGE);
		break;
	case LOW_POWER:
		err = mdfld_dsi_send_mcs_short_lp(sender,
					(u8)dbgfs.addr, (u8)arg, 1,
					MDFLD_DSI_SEND_PACKAGE);
		break;
	}

	if (err) {
		DRM_ERROR("%s: %d: error\n",
		__func__, __LINE__);
		ret = -1;
	}

	/* releasing display and MIPI bus */
	if ((type == HIGH_SPEED) || (type == LOW_POWER)) {
		mdfld_dsi_dsr_allow_locked(dbgfs_dsi_config);
		power_island_put(power_island);
	}

exit_dbgfs_write:
	kfree(str);
	return ret;
}

static ssize_t dbgfs_addr_write(struct file *file, const char __user *buff,
				size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int err = 0;

	ret = count;

	if (*ppos < 0 || !count)
		return -EINVAL;

	err = dbgfs_write(buff, count, ADDR);
	if (err < 0)
		return err;

	*ppos += ret;

	return ret;
}

static ssize_t dbgfs_send_hs_write(struct file *file, const char __user *buff,
				    size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int err = 0;

	ret = count;

	if (*ppos < 0 || !count)
		return -EINVAL;

	err = dbgfs_write(buff, count, HIGH_SPEED);
	if (err < 0)
		return err;

	*ppos += ret;

	return count;
}

static ssize_t dbgfs_send_lp_write(struct file *file, const char __user *buff,
				    size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	int err = 0;

	ret = count;

	if (*ppos < 0 || !count)
		return -EINVAL;

	err = dbgfs_write(buff, count, LOW_POWER);
	if (err < 0)
		return err;

	*ppos += ret;

	return count;
}

#ifdef INNOLUX_DEBUG
static
int readback_initcode(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	int i;
	u8 *cmd, *arg;
	u8 data;

	DRM_INFO("%s\n", __func__);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	msleep(120);

	cmd = (u8 *)innolux_init_code;
	arg = cmd + 1;

	for (i = 0; i < ARRAY_SIZE(innolux_init_code); i++) {
		switch (*cmd) {
		case 0xFF:
			err = mdfld_dsi_send_mcs_short_lp(sender,
					*cmd, *arg, 1,
					MDFLD_DSI_SEND_PACKAGE);
			break;
		default:
			err = mdfld_dsi_read_mcs_lp(sender, *cmd, &data, 1);
				DRM_ERROR("%01X %01X\n", *cmd, data);
			break;
		}
		cmd += 2;
		arg += 2;
	}

readback_err:
	if (err) {
		err = -EIO;
		DRM_ERROR("innolux: error in readback loop\n");
	}
	return err;
}
#endif
