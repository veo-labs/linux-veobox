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

static int adv7611_disable_mute_conditions(struct snd_soc_codec * codec)
{
	unsigned int reg;

	/* Clean mute condition masks */
	snd_soc_write(codec, ADV7611_MUTE_MASK_21_16, 0x00);
	snd_soc_write(codec, ADV7611_MUTE_MASK_15_8, 0x00);
	snd_soc_write(codec, ADV7611_MUTE_MASK_7_0, 0x00);

	/* No mute automatic conditions, disabling Audio Delay Line */
	reg = snd_soc_read(codec, ADV7611_AUDIO_MUTE_SPEED);
	snd_soc_write(codec, ADV7611_AUDIO_MUTE_SPEED,
			(reg & 0x1f) | 0xc0);


	return 0;
}

static struct regmap * adv7611_get_regmap(struct device * dev)
{
	struct adv76xx_snd_data * snd_data = dev->platform_data;

	if (snd_data == NULL)
		return NULL;

	return snd_data->regmap;
}


static int adv7611_codec_probe(struct snd_soc_codec * codec){

	/* Disable mute conditions */
	adv7611_disable_mute_conditions(codec);

	/* Setting MCLK to 256Fs */
	snd_soc_write(codec, ADV7611_MCLK_FS, 0x01);

	return 0;
}


static struct snd_soc_codec_driver adv7611_audio_codec = {
	/* We can define here */
	.probe				= adv7611_codec_probe,
	.get_regmap			= adv7611_get_regmap,

	/*
	.suspend			= ,
	.resume				= ,
	.component_driver	= , // struct snd_soc_component_driver

	// Default control and setup, added after probe() is run
	.controls			= , // snd_kcontrol_new
	.num_controls		= , // int
 	.dapm_widgets		= , // snd_soc_dapm_widget
	.num_dapm_widgets	= , // int
	.dapm_routes		= , // snd_soc_dapm_route
	.num_dapm_routes	= , // int

	// codec wide operations
	.set_sysclk			= ,
	.set_pll			= ,

	// codec IO
	.read				= ,
	.write				= ,
	.reg_cache_size		= , // unsigned int
	.reg_cache_step		= , // short
	.reg_word_size		= , // short
	.reg_cache_default	= ,

	// codec bias level
	.set_bias_level		= ,
	.idle_bias_off		= , // bool
	.suspend_bias_off 	= , // bool

	.seq_notifier		= ,
	 */
};

static int adv76xx_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	struct snd_soc_codec *codec = dai->codec;
	int finalfmt = snd_soc_read(codec, ADV7611_HDMI_REGISTER_03);

	/* setting i2s data format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		finalfmt &= (~ADV7611_I2S_OUT_MODE_MASK | ADV7611_I2S_OUT_MODE_I2S);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ADV7611_HDMI_REGISTER_03, finalfmt);

	return 0;

}

static int adv7611_ops_hw_params(struct snd_pcm_substream * substream,
		struct snd_pcm_hw_params * hw_params, struct snd_soc_dai * dai)
{

	struct snd_soc_codec *codec = dai->codec;
	unsigned int reg;
	unsigned int channels = params_channels(hw_params);
	unsigned int rates = params_rate(hw_params);
	int linewidthreg = snd_soc_read(codec, ADV7611_LINE_WIDTH_1);

	// Force N update
	reg = snd_soc_read(codec, ADV7611_REGISTER_5AH);
	snd_soc_write(codec, ADV7611_REGISTER_5AH,
			reg | 0x01);

	// Get the Audio PLL Locked status
	reg = snd_soc_read(codec, ADV7611_HDMI_REGISTER_04) & ADV7611_AUDIO_PLL_LOCKED;
	dev_info(dai->dev, "Audio PLL Locked is %s \n", reg ? "locked" : "not locked");

	// Read Mute value
	//reg = regmap_read(codec, ADV7611_MUTE_CTRL) & ADV7611_AUDIO_PLL_LOCKED;

	// Read the CTS from the data stream 5b, 5c ,5d
	dev_info(dai->dev, "ADV7611 Audio Codec Ops hw params: fifo size %lu rate %u rate deno %u channels %u\n" ,
			(unsigned long)hw_params->fifo_size,rates , hw_params->rate_den, hw_params->mres, channels );

	// Read the N from the data stream 5d, 5e, 5f

	return 0;
}

static void adv7611_ops_shutdown(struct snd_pcm_substream * substream,
		struct snd_soc_dai * dai)
{

	dev_info(dai->dev, "ADV7611 Audio Codec Ops shutdown\n");
}


static int adv7611_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
	unsigned int freq_in, unsigned int freq_out)
{
	dev_info(dai->dev, "Audio Codec Driver set_pll clkid %d source %d freq_in %d freq_out %d\n", pll_id, source, freq_in, freq_out);

	return 0;
}

static int adv7611_set_sysclk(struct snd_soc_dai *dai,
		  int clk_id, int source, unsigned int freq, int dir)
{

	dev_info(dai->dev, "Audio Codec Driver set_sysclk clkid %d source %d freq %d dir %d\n", clk_id, source, freq, dir);

	return 0;
}


static struct snd_soc_dai_ops adv7611_dai_ops = {

	// ALSA PCM audio operations
	.shutdown		= adv7611_ops_shutdown,
	.hw_params		= adv7611_ops_hw_params,
		//.hw_free		= ,
		//.prepare		= ,
	.set_fmt		= adv76xx_set_dai_fmt,
	.set_sysclk		= adv7611_set_sysclk,
	.set_pll		= adv7611_set_pll,

	/* We can define here DAI clocking configuration

	.set_clkdiv		= ,
	.set_bclk_ratio	= ,

	 //DAI format configuration
	.xlate_tdm_slot_mask	= ,
	.set_tdm_slot			= ,
	.set_channel_map		= ,
	.set_tristate			= ,

	// Digital Mute
	.digital_mute	= ,
	.mute_stream	= ,
*/
};

#define ADV7611_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)

// DAI (Digital Audio Interface) struct for the codec
static struct snd_soc_dai_driver adv7611_dai = {
	.name 	= DAI_DRIVER_NAME, // Same used on snd_soc_dai_link.codec_dai_name
	.capture = {
			.stream_name = "HDMI-Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_96000,
			.formats = ADV7611_FORMATS,
		},
	.ops 	= &adv7611_dai_ops, // Operations
};


/*
 *
 */
static int adv76xx_codec_dev_probe(struct platform_device *pdev)
{
	int ret;

	// Check good pdev value
	if (pdev == NULL )
		return -ENODEV;

	pdev->dev.init_name = pdev->name;

	// Register the codec on the platform device
	ret = snd_soc_register_codec(&pdev->dev,
			&adv7611_audio_codec,
			&adv7611_dai,
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

static const struct platform_device_id adv76xx_id[] = {
	{"adv7611-asoc-codec", 0},
	{"adv7604-asoc-codec", 0},
};

static const struct of_device_id adv76xx_dt_ids[] = {
	{ .compatible = "adi,adv76xx", },
	{ }
};
MODULE_DEVICE_TABLE(of, adv76xx_dt_ids);

static struct platform_driver adv76xx_snd_driver = {
	.driver	= {
		.name	= PLATFORM_DRIVER_NAME, // Same name used on platform_device_register_resndata
		.owner	= THIS_MODULE,
		.of_match_table = adv76xx_dt_ids,
	},
	.probe 	= adv76xx_codec_dev_probe,
	.remove	= adv76xx_codec_dev_remove,
	.id_table = adv76xx_id,
};

module_platform_driver(adv76xx_snd_driver);

MODULE_AUTHOR("Pablo Anton <pablo.antond@vodalys-labs.com>");
MODULE_AUTHOR("Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>");
MODULE_DESCRIPTION("ADV7611 Audio Codec Driver");
MODULE_LICENSE("GPL");
