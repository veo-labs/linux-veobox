/*
 * Copyright (C) 2015 Vodalys-labs
 * Author: Pablo Anton <pablo.anton@vodalys-labs.com>
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
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>

#include "../codecs/sgtl5000.h"

#define UNDEFINED -1

/* TODO Create header file for those variables */
#define	VBX3_FPGA_REG_VERSION			0x00
#define	VBX3_FPGA_REG_CTRL_CHAN0		0x01
#define	VBX3_FPGA_REG_CTRL_CHAN1		0x02
#define	VBX3_FPGA_REG_CTRL_PATTERN_CHAN0	0x03
#define	VBX3_FPGA_REG_GLOBAL_STATUS		0x04
#define	VBX3_FPGA_REG_STATUS_SDI0		0x05
#define	VBX3_FPGA_REG_STATUS_SDI1		0x06
#define	VBX3_FPGA_REG_EVENT_CHAN0		0x07
#define	VBX3_FPGA_REG_EVENT_CHAN1		0x08

#define DAI_FMT_DEFAULT (SND_SOC_DAIFMT_I2S |\
			SND_SOC_DAIFMT_NB_NF |\
			SND_SOC_DAIFMT_CBM_CFM)

/**
 * CODEC private data
 *
 * @mclk_freq: Clock rate of MCLK
 * @mclk_id: MCLK (or main clock) id for set_sysclk()
 * @fll_id: FLL (or secordary clock) id for set_sysclk()
 * @pll_id: PLL id for set_pll()
 */
struct codec_priv {
	unsigned long mclk_freq;
	u32 mclk_id;
	u32 fll_id;
	u32 pll_id;
};

struct vbx_aswitch_state {
	struct platform_device *pdev;
	struct regmap *regmap;
	int instance;
	int n_links;
	struct snd_soc_dai_link * dais;
	struct snd_soc_card card;
	struct codec_priv *codec_priv;
};

static struct vbx_aswitch_state *get_state_from_stream(
		struct snd_pcm_substream *substream)
{
	struct snd_pcm *pcm = substream->pcm;
	struct snd_soc_card *soc_card;
	struct vbx_aswitch_state * state;

	if (!pcm || !pcm->card) {
		printk(KERN_ERR "Error getting driver data\n");
		return NULL;
	}

	soc_card = dev_get_drvdata(pcm->card->dev);

	if (!soc_card) {
		dev_err(pcm->card->dev, "Error getting driver data, no soc card\n");
		return NULL;
	}

	state = snd_soc_card_get_drvdata(soc_card);

	return state;
}

static int startup_ops(struct snd_pcm_substream *substream)
{
	struct vbx_aswitch_state *state = get_state_from_stream(substream);
	unsigned int value;

	if (!state)
		return -EINVAL;

	switch (state->instance) {
	case 0:
		switch (substream->pcm->device) {
		case 0: /* ADV7611 */
			regmap_read(state->regmap, VBX3_FPGA_REG_CTRL_CHAN0, &value);
			regmap_write(state->regmap,
				     VBX3_FPGA_REG_CTRL_CHAN0,
				     value & 0xf9);

			break;
		case 1: /* SGTL5000 */
			regmap_read(state->regmap, VBX3_FPGA_REG_CTRL_CHAN0, &value);
			regmap_write(state->regmap,
				     VBX3_FPGA_REG_CTRL_CHAN0,
				     value | 0x04);

			break;
		case 2: /* SDI0 */
			regmap_read(state->regmap, VBX3_FPGA_REG_CTRL_CHAN0, &value);
			regmap_write(state->regmap,
				     VBX3_FPGA_REG_CTRL_CHAN0,
				     (value & 0xfb) | 0x02);
			break;
		default:
			return -EINVAL;
		};
		break;
	case 1:
		switch (substream->pcm->device) {
		case 0: /* ADV7604 */
			regmap_read(state->regmap, VBX3_FPGA_REG_CTRL_CHAN1, &value);
			regmap_write(state->regmap,
				     VBX3_FPGA_REG_CTRL_CHAN1,
				     value & 0xf9);
			break;
		case 1: /* SDI1 */
			regmap_read(state->regmap, VBX3_FPGA_REG_CTRL_CHAN1, &value);
			regmap_write(state->regmap,
				     VBX3_FPGA_REG_CTRL_CHAN1,
				     (value & 0xfb) | 0x02);
			break;
		default:
			return -EINVAL;
		};
		break;
	default:
		return -EINVAL;
	};

	return  0;
}

static const struct snd_soc_ops dai_ops = {
	.startup = &startup_ops,
};

/* Configure private properties (clocks) of codecs if any */
static int vbx_aswitch_late_probe(struct snd_soc_card *card)
{
	struct vbx_aswitch_state *state = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai;
	int ret = 0, i;

	if (!state)
		return -ENODEV;

	for (i = 0 ; i < card->num_rtd ; i++) {

		codec_dai = card->rtd[i].codec_dai;

		if (!strcmp(codec_dai->name, state->dais[i].codec_dai_name) &&
				(state->codec_priv[i].mclk_id != UNDEFINED)) {

			ret = snd_soc_dai_set_sysclk(codec_dai,
				state->codec_priv[i].mclk_id,
				state->codec_priv[i].mclk_freq,
				SND_SOC_CLOCK_IN);

			if (ret)
				dev_err(card->dev, "failed setting sysclk for %s\n", codec_dai->name);
		}
	}

	return 0;
}

static int configure_i2c_dev_clock(struct vbx_aswitch_state *state,
			  int dai_index,
			  struct device_node *codec)
{
	struct i2c_client *client;
	struct clk *dev_clk;

	/* Configure clock */
	client = of_find_i2c_device_by_node(codec);
	if (!client) {
		dev_dbg(&state->pdev->dev,
				"There is no i2c client associated to %s\n",
				codec->name);
		return -ENODEV;
	}


	/* Get the MCLK rate only, and leave it controlled by CODEC drivers */
	dev_clk = clk_get(&client->dev, NULL);
	if (IS_ERR(dev_clk))
		return -ENODEV;

	state->codec_priv[dai_index].mclk_freq = clk_get_rate(dev_clk);
	clk_put(dev_clk);

	return 0;
}

static int parse_dt(struct vbx_aswitch_state *state)
{
	struct device *dev = &state->pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *codec = NULL, *cpu_np, *child_node;
	const char *name, *stream_name, *dai_name, *codec_name;
	int err;
	u32 index, clock_id;

	if (!node) {
		dev_err(dev, "Platform has not device node\n");
		return -EINVAL;
	}

	/* Getting the number of dai-links for this sound card*/
	state->n_links = of_get_child_count(node);

	/* Getting instance number */
	err = of_property_read_u32(node, "instance", &state->instance);
	if (err < 0) {
		dev_err(dev, "Could not find instance property: %d\n",
				err);
		return err;
	}

	/* Getting ssi controller as cpu dai */
	cpu_np = of_parse_phandle(node, "ssi-controller", 0);
	if (!cpu_np){
		dev_err(dev, "No ssi controller found\n");
		return -ENODEV;
	}

	/* Initialize DAI structure and the private structure */
	state->dais = devm_kzalloc(dev,
			sizeof(struct snd_soc_dai_link) * state->n_links,
			GFP_KERNEL);
	if (!state->dais)
		return -ENOMEM;

	state->codec_priv = devm_kzalloc(dev,
				sizeof(struct codec_priv) * state->n_links,
				GFP_KERNEL);
	if (!state->codec_priv)
		goto fail_malloc;

	/* For each links complete a dai_link information */
	for_each_child_of_node(node, child_node) {

		err = of_property_read_u32(child_node, "index", &index);
		if (err < 0){
			dev_err(dev, "Node has no index property\n");
			goto fail;
		}

		err = of_property_read_string(child_node, "link-name",
				(const char **)&name);
		if (err < 0){
			dev_err(dev, "Node has no link-name property\n");
			goto fail;
		}

		err = of_property_read_string(child_node, "stream-name",
				(const char **)&stream_name);
		if (err < 0){
			dev_err(dev, "Node has no stream-name property\n");
			goto fail;
		}

		err = of_property_read_string(child_node, "codec-dai-name",
				(const char **)&dai_name);
		if (err < 0){
			dev_err(dev, "Node has no codec-dai-name property\n");
			goto fail;
		}


		err = of_property_read_string(child_node, "codec-name",
				(const char **)&codec_name);
		if (err < 0){
			codec = of_parse_phandle(child_node, "codec", 0);
			if (!codec){
				dev_err(dev,
					"Neither codec-name nor codec property\n");
				goto fail;
			}

			state->dais[index].codec_of_node = codec;

		} else {
			state->dais[index].codec_name = codec_name;
		}

		/* If property codec-clock exists, we should configure it */
		err = of_property_read_u32(child_node, "codec-clock-id", &clock_id);
		if (!err && codec){
			if (!configure_i2c_dev_clock(state, index, codec))
				state->codec_priv[index].mclk_id =
						clock_id;
		} else {
			state->codec_priv[index].mclk_id = UNDEFINED;
		}

		dev_dbg(dev, "Setting dai %d:\n", index);
		dev_dbg(dev, "\tLink name %s:\n", name);
		dev_dbg(dev, "\tStream name %s:\n", stream_name);
		dev_dbg(dev, "\tCodec dai name %s:\n", dai_name);
		dev_dbg(dev, "\tCodec name %s:\n", codec_name);

		state->dais[index].codec_dai_name = dai_name;
		state->dais[index].stream_name = stream_name;
		state->dais[index].name = name;
		state->dais[index].dai_fmt = DAI_FMT_DEFAULT;
		state->dais[index].ops = &dai_ops;


		state->dais[index].cpu_of_node = cpu_np;
		state->dais[index].platform_of_node = cpu_np;

		of_node_put(child_node);
	}

	return 0;
fail:
	of_node_put(child_node);
	devm_kfree(&state->pdev->dev,state->codec_priv);
fail_malloc:
	devm_kfree(&state->pdev->dev,state->dais);
	return err;
}

static int vbx_aswitch_probe(struct platform_device *pdev)
{
	struct vbx_aswitch_state *state;
	int err;
	char card_name[32];

	dev_dbg(&pdev->dev, "Starting %s probe\n", pdev->name);

	/* Alloc state memory */
	state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->pdev = pdev;
	state->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!state->regmap)
		return -ENODEV;

	/* Parsing Devicetree node */
	err = parse_dt(state);
	if (err) {
		dev_err(&pdev->dev, "Error parsing DT Links\n");
		return err;
	}

	/* Create card name depending on instance number */
	sprintf(card_name, "snd-card-%d", state->instance);

	/* Initialize sound card */
	state->card.dev = &pdev->dev;
	state->card.name = card_name;
	state->card.dai_link = state->dais;
	state->card.num_links = state->n_links;
	state->card.late_probe = &vbx_aswitch_late_probe;

	snd_soc_card_set_drvdata(&state->card, state);

	err = devm_snd_soc_register_card(&pdev->dev, &state->card);
	if (err){
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", err);
		return -EPROBE_DEFER;
	}

	dev_dbg(&pdev->dev, "device probed\n");

	return 0;
}

static int vbx_aswitch_remove(struct platform_device *pdev)
{

	return 0;
}

static struct of_device_id vbx_aswitch_dt_id[] = {
	{ .compatible = "vbx,aswitch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vbx_aswitch_dt_id);

static struct platform_driver vbx_vswitch_pdrv = {
	.probe		= vbx_aswitch_probe,
	.remove		= vbx_aswitch_remove,
	.driver		= {
		.name	= "vbx_aswitch",
		.owner	= THIS_MODULE,
		.of_match_table	= vbx_aswitch_dt_id,
	},
};

module_platform_driver(vbx_vswitch_pdrv);
MODULE_ALIAS("platform:vbx_aswitch");
MODULE_DESCRIPTION("Audio switch device driver for Veobox FPGA");
MODULE_AUTHOR("Pablo Anton");
MODULE_LICENSE("GPL");
