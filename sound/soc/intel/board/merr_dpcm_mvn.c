/*
 *  ASoc DPCM Machine driver for Intel Marvin MID platform
 *
 *  Copyright (C) 2014-2015 Intel Corp
 *  Author: Michael Soares <michaelx.soares@intel.com>
 *  Author: Mattijs Korpershoek <mattijsx.korpershoek@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

struct mvn_slot_info {
	unsigned int tx_mask;
	unsigned int rx_mask;
	int slots;
	int slot_width;
};

#ifdef CONFIG_PM_SLEEP
static int snd_merr_dpcm_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}

static void snd_merr_dpcm_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return;
}

static int snd_merr_dpcm_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}
#else
#define snd_merr_dpcm_prepare NULL
#define snd_merr_dpcm_complete NULL
#define snd_merr_dpcm_poweroff NULL
#endif

const struct dev_pm_ops snd_merr_dpcm_mc_pm_ops = {
	.prepare = snd_merr_dpcm_prepare,
	.complete = snd_merr_dpcm_complete,
	.poweroff = snd_merr_dpcm_poweroff,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

#define CONFIG_SLOT(slot_tx_mask, slot_rx_mask, num_slot, width)\
	(struct mvn_slot_info){ .tx_mask = slot_tx_mask,\
				  .rx_mask = slot_rx_mask,\
				  .slots = num_slot,\
				  .slot_width = width, }

static int merr_mvn_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops merr_mvn_ops = {
		.startup = merr_mvn_startup,
};

static int merr_mvn_set_slot_and_format(struct snd_soc_dai *dai,
			struct mvn_slot_info *slot_info, unsigned int fmt)
{
	int ret;

	ret = snd_soc_dai_set_tdm_slot(dai, slot_info->tx_mask,
		slot_info->rx_mask, slot_info->slots, slot_info->slot_width);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	return ret;
}

static int ssp0_config_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	int ret = 0;
	unsigned int fmt;
	struct mvn_slot_info *info;

	pr_err("Invoked %s for dailink %s\n", __func__, dai_link->name);

	/* I2S | framesync active high | master mode */
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBS_CFS;
	/* tx_mask = 3 | rx_mask = 3 | 2 slots | 24 bits */
	info = &CONFIG_SLOT(0x3, 0x3, 2, SNDRV_PCM_FORMAT_S24_LE);

	ret = merr_mvn_set_slot_and_format(dai, info, fmt);

	return ret;
}

static const struct snd_soc_pcm_stream ssp0_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_16000,
	.rate_max = SNDRV_PCM_RATE_16000,
	.channels_min = 2,
	.channels_max = 2,
};

struct snd_soc_dai_link merr_msic_dailink[] = {
	/* front ends */
	[MERR_DPCM_AUDIO] = {
		.name = "Media Audio Port",
		.stream_name = "Marvin Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &merr_mvn_ops,
	},
	/* back-end <-> back-end link */
	{
		.name = "DMIC-Loop Port",
		.stream_name = "Marvin DMIC-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &ssp0_dai_params,
		.be_fixup = ssp0_config_fixup,
		.dsp_loopback = true,
	},
	/* back-ends */
	{
		.name = "SSP0-DMIC",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
};

static const struct snd_soc_dapm_route map[] = {
	{ "ssp0 Tx", NULL, "modem_out"},
	{ "modem_in", NULL, "ssp0 Rx" },
};

/* SoC card */
static struct snd_soc_card snd_soc_card_merr = {
	.name = "marvin-audio",
	.dai_link = merr_msic_dailink,
	.num_links = ARRAY_SIZE(merr_msic_dailink),
	.dapm_routes = map,
	.num_dapm_routes = ARRAY_SIZE(map),
};

static int snd_merr_dpcm_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	pr_debug("%s enter\n", __func__);

	/* register the soc card */
	snd_soc_card_merr.dev = &pdev->dev;
	ret_val = snd_soc_register_card(&snd_soc_card_merr);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_merr);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static int snd_merr_dpcm_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	pr_err("snd_merr_dpcm_remove");
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver snd_merr_dpcm_drv = {
	.driver = {
			.owner = THIS_MODULE,
			.name = "merr_dpcm_mvn",
			.pm = &snd_merr_dpcm_mc_pm_ops,
	},
	.probe = snd_merr_dpcm_probe,
	.remove = snd_merr_dpcm_remove,
};
module_platform_driver(snd_merr_dpcm_drv);

MODULE_DESCRIPTION("ASoC Intel(R) Marvin MID Machine driver");
MODULE_AUTHOR("Michael Soares <michaelx.soares@intel.com>");
MODULE_AUTHOR("Mattijs Korpershoek <mattijsx.korpershoek@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:merr_dpcm_mvn");
