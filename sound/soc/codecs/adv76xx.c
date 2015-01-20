 /*
 * Copyright (C) 2014 Vodalys-labs
 * Author: Pablo Anton <pablo.anton@vodalys-labs.com>
 * Author: Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <linux/device.h>
#include <media/adv7604.h>

#include "adv76xx.h"

static int adv76xx_cnf_mute_conditions(struct snd_soc_codec *codec)
{
	unsigned int reg;

	/* Clean mute condition masks */
	snd_soc_write(codec, ADV76XX_MUTE_MASK_21_16, 0x00);
	snd_soc_write(codec, ADV76XX_MUTE_MASK_15_8, 0x00);
	snd_soc_write(codec, ADV76XX_MUTE_MASK_7_0, 0x00);

	/* Configure audio mute speed */
	snd_soc_write(codec, ADV76XX_AUDIO_MUTE_SPEED, 0x0f);

	/* Configure how to unmute */
	reg = snd_soc_read(codec, ADV76XX_MUTE_CTRL);
	snd_soc_write(codec, ADV76XX_MUTE_CTRL,
			(reg & 0xe0));

	return 0;
}

static struct regmap * adv76xx_get_regmap(struct device *dev)
{
	struct adv76xx_snd_data * snd_data = dev->platform_data;

	if (snd_data == NULL)
		return NULL;

	return snd_data->state->regmap[ADV7604_PAGE_HDMI];
}

/*
 * Set the Master Clock register multiplier that control
 * the Master Clock output frequency
 */
static int adv76xx_set_mclk_fs_n(struct snd_soc_codec *codec,
				 unsigned int fs)
{
	struct adv76xx_snd_data * snd_data = codec->dev->platform_data;

	if (!snd_data)
		return -ENODEV;

	return regmap_write(snd_data->state->regmap[ADV7604_PAGE_AFE],
			ADV76XX_MCLK_FS, fs);
}

static int adv76xx_codec_probe(struct snd_soc_codec * codec){

	/* Do not mux SPDIF and I2S output */
	snd_soc_write(codec, ADV7611_DST_MAP_ROT_2_0, 0x04);

	/* Configure mute conditions */
	adv76xx_cnf_mute_conditions(codec);

	/* Set MCLK to 256fs*/
	adv76xx_set_mclk_fs_n(codec, 0x01);

	return 0;
}

static struct snd_soc_codec_driver adv76xx_audio_codec = {
	.probe = adv76xx_codec_probe,
	.get_regmap = adv76xx_get_regmap,
};

static int adv76xx_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct adv76xx_snd_data * snd_data = dai->dev->platform_data;
	struct snd_soc_codec *codec = dai->codec;
	int finalfmt = snd_soc_read(codec, ADV76XX_HDMI_REGISTER_03);

	/* Setting i2s data format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		finalfmt &= (~ADV76XX_I2SOUTMODE_MASK |
				ADV76XX_I2SOUTMODE_I2S);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ADV76XX_HDMI_REGISTER_03, finalfmt);

	/*
	 * i2s clock and frame master setting.
	 * ONLY support:
	 *  - clock and frame master
	*/
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* Setting MCLK to 256fs */
		adv76xx_set_mclk_fs_n(codec, 0x01);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void log_pcm_sample_info(struct snd_soc_dai *dai)
{
	int freq;
	unsigned int reg;
	struct adv76xx_snd_data * snd_data = dai->dev->platform_data;

	/* Check channel status data */
	if (snd_data) {
		regmap_read(snd_data->state->regmap[ADV7604_PAGE_IO], 0x65, &reg);
		reg &= 0x80;
		dev_info(dai->dev, "Channel status data is %s\n", reg ? "valid" : "not valid");
	}

	/* Check if there are PCM audio samples */
	reg = snd_soc_read(dai->codec, 0x18) & 0x01;
	dev_info(dai->codec->dev, "Received PCM package: %s\n", reg ? "yes":"no");

	reg = snd_soc_read(dai->codec, 0x36) & 0x02;
	dev_info(dai->dev, "Received PCM package(from CS_DATA): %s\n", reg ? "no":"yes");

	if (!reg) {
		/* Get audio sampling frequency */
		reg = snd_soc_read(dai->codec, 0x39) & 0x0f;
		dev_info(dai->dev, "Sample Freq: %x\n", reg);

		reg = snd_soc_read(dai->codec, 0x39) & 0x30;
		dev_info(dai->codec->dev, "Clock accuracy: %x\n", reg);

		reg = snd_soc_read(dai->codec, 0x3a) & 0x01;
		dev_info(dai->dev, "Max audio sample word length: %d\n", reg ? 24 : 20);

		reg = snd_soc_read(dai->codec, 0x3a) & 0x0e;
		dev_info(dai->dev, "Audio sample word length: %x\n", reg);
	}

	reg = snd_soc_read(dai->codec, 0x7e);
	dev_info(dai->dev, "FIFO Underflow %s\n", reg & 0x40 ? "true": "false");
	dev_info(dai->dev, "FIFO Overflow %s\n", reg & 0x20 ? "true": "false");
}

static int adv76xx_ops_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int reg;
	int err = 0;
	int linewidthreg = snd_soc_read(codec, ADV76XX_LINE_WIDTH_1);
	struct adv76xx_snd_data * snd_data = codec->dev->platform_data;

	if (!snd_data)
		return -ENODEV;

	/* Check if there are PCM audio samples */
	reg = snd_soc_read(dai->codec, ADV7611_PACKETS_DETECTED_2);
	if (reg < 0 || !(reg & ADV76XX_AUDIO_SAMPLE_PCKT_DET))
		dev_dbg(dai->dev, "Warning: It seems no PCM Audio available\n");

	/* Check if there Audio is muted */
	err = regmap_read(snd_data->state->regmap[ADV7604_PAGE_IO],
			ADV7611_HDMI_LVL_RAW_STATUS_2, &reg);
	if (!err && ((reg & 0x40) || (reg & 0x20))) {
		dev_dbg(dai->dev, "Warning: It seems AV Mute or Internal Audio is muted\n");
	}

	return 0;
}

static int adv76xx_mute(struct snd_soc_dai *dai, int mute)
{
	int reg, err;
	struct snd_soc_codec *codec = dai->codec;

	/* Set general MUTE_AUDIO depending on parameter mute */
	reg = snd_soc_read(codec, ADV76XX_MUTE_CTRL);

	if (reg < 0)
		return -1;

	if (snd_soc_write(codec, ADV76XX_MUTE_CTRL,
		      (reg & (mute ? 0xff : 0xfe))))
		return -1;

	return 0;
}

static struct snd_soc_dai_ops adv76xx_dai_ops = {
	.hw_params = adv76xx_ops_hw_params,
	.set_fmt = adv76xx_set_dai_fmt,
	.digital_mute = adv76xx_mute,
};

#define ADV76XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)

/* DAI (Digital Audio Interface) struct for the codec */
static struct snd_soc_dai_driver adv76xx_dais[] = {
	{
		.name = DAI_ADV7604_NAME,
		.capture = {
			.stream_name    = "HDMI-Capture",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= ADV76XX_FORMATS,
		},
		.ops = &adv76xx_dai_ops,
	},
	{
		.name = DAI_ADV7611_NAME,
		.capture = {
			.stream_name    = "HDMI-Capture",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= ADV76XX_FORMATS,
		},
		.ops = &adv76xx_dai_ops,
	},
};

static int adv76xx_codec_dev_probe(struct platform_device *pdev)
{
	int ret, index;
	struct adv76xx_snd_data * snd_data = pdev->dev.platform_data;

	/* Check platform data value */
	if (!snd_data)
		return -EINVAL;

	/* Register the codec on the platform device */
	ret = snd_soc_register_codec(&pdev->dev,
			&adv76xx_audio_codec,
			&adv76xx_dais[snd_data->state->info->type],
			1);

	if (ret)
		return -ENODEV;

	return ret;
}

static int adv76xx_codec_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct platform_device_id adv76xx_ids[] = {
	{"adv7611-asoc-codec", 0},
	{"adv7604-asoc-codec", 0},
	{},
};
MODULE_DEVICE_TABLE(platform, adv76xx_ids);

static struct platform_driver adv76xx_snd_driver = {
	.driver	= {
		.name   = PLATFORM_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe 	= adv76xx_codec_dev_probe,
	.remove	= adv76xx_codec_dev_remove,
	.id_table = adv76xx_ids,
};

module_platform_driver(adv76xx_snd_driver);

MODULE_AUTHOR("Pablo Anton <pablo.antond@vodalys-labs.com>");
MODULE_AUTHOR("Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>");
MODULE_DESCRIPTION("ADV7611 Audio Codec Driver");
MODULE_LICENSE("GPL");
