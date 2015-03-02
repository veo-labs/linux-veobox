/*
 * Copyright (C) 2015 Vodalys-labs
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
#include <linux/platform_device.h>
#include <sound/soc.h>

static int vbx3_sdi_codec_probe(struct snd_soc_codec * codec)
{
	dev_dbg(codec->dev, "Into %s\n", __func__);

	return 0;
}

static struct snd_soc_codec_driver vbx3_sdi_audio_codec = {
	.probe = vbx3_sdi_codec_probe,
};

static int vbx3_sdi_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	dev_dbg(dai->dev, "Into %s\n", __func__);

	/* Check i2s data format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	/*
	 * i2s clock and frame master setting.
	 */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vbx3_sdi_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params,
				 struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "Into %s\n", __func__);

	return 0;
}

static struct snd_soc_dai_ops vbx3_sdi_dai_ops = {
	.hw_params = vbx3_sdi_hw_params,
	.set_fmt = vbx3_sdi_set_dai_fmt,
};


#define SDI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

/* DAI (Digital Audio Interface) driver struct for the codec */
static struct snd_soc_dai_driver vbx3_sdi_dai[] = {
	{
		.name = "vbx3-sdi0",
		.capture = {
			.stream_name	= "SDI-Capture",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SDI_FORMATS,
		},
		.ops = &vbx3_sdi_dai_ops,
	},
	{
		.name = "vbx3-sdi1",
		.capture = {
			.stream_name	= "SDI-Capture",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_48000,
			.formats	= SDI_FORMATS,
		},
		.ops = &vbx3_sdi_dai_ops,
	},

};


static int vbx3_sdi_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int err;
	u32 reg;

	dev_dbg(&pdev->dev, "Into SDI Probe\n");

	/* Check node value */
	if (!node) {
		dev_err(&pdev->dev, "No DT Node available on %s\n", pdev->name);
		return -EINVAL;
	}

	/* Get reg number from device tree */
	err = of_property_read_u32(node, "index", &reg);
	if (err < 0) {
		dev_err(&pdev->dev, "Could not find instance number : %d\n",
				err);
		return err;
	}

	/* Register the codec on the platform device */
	err = snd_soc_register_codec(&pdev->dev,
			&vbx3_sdi_audio_codec,
			&vbx3_sdi_dai[reg],
			1);



	if (err) {
		dev_err(&pdev->dev, "Error registering SDI Codec\n");
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "SDI Codec register\n");

	return err;
}

static int vbx3_sdi_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct of_device_id vbx3_sdi_dt_id[] = {
	{ .compatible = "vbx,sdisound" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vbx3_sdi_dt_id);

static struct platform_driver vbx3_sdi_snd_driver = {
	.driver	= {
		.name   = "sdi-asoc",
		.owner	= THIS_MODULE,
		.of_match_table = vbx3_sdi_dt_id,
	},
	.probe 	= vbx3_sdi_probe,
	.remove	= vbx3_sdi_remove,
};

module_platform_driver(vbx3_sdi_snd_driver);

MODULE_AUTHOR("Pablo Anton <pablo.antond@vodalys-labs.com>");
MODULE_AUTHOR("Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>");
MODULE_DESCRIPTION("Veobox FPGA SDI Audio Codec Driver");
MODULE_LICENSE("GPL");
