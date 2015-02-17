/*
 * LMH0395 SPI driver.
 * Copyright (C) 2014  Jean-Michel Hautbois
 *
 * 3G HD/SD SDI Dual Output Low Power Extended Reach Adaptive Cable Equalizer
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>

#define VBX_VSWITCH_SDI_INPUT	0
#define VBX_VSWITCH_HDMI_INPUT	1
#define VBX_VSWITCH_OUTPUT	2
#define VBX_VSWITCH_PADS_NUM	3

#define	VBX3_FPGA_REG_VERSION			0x00
#define	VBX3_FPGA_REG_CTRL_CHAN0		0x01
#define	VBX3_FPGA_REG_CTRL_CHAN1		0x02
#define	VBX3_FPGA_REG_CTRL_PATTERN_CHAN0	0x03
#define	VBX3_FPGA_REG_GLOBAL_STATUS		0x04
#define	VBX3_FPGA_REG_STATUS_SDI0		0x05
#define	VBX3_FPGA_REG_STATUS_SDI1		0x06
#define	VBX3_FPGA_REG_EVENT_CHAN0		0x07
#define	VBX3_FPGA_REG_EVENT_CHAN1		0x08

struct vbx_vswitch_state {
	struct platform_device *pdev;
	struct v4l2_subdev sd;
	struct media_pad pads[VBX_VSWITCH_PADS_NUM];
	struct regmap *regmap;
	unsigned int instance;
};

static inline struct vbx_vswitch_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vbx_vswitch_state, sd);
}

static int vbx_vswitch_log_status(struct v4l2_subdev *sd)
{
	struct vbx_vswitch_state *state = to_state(sd);
	unsigned int value, aux;

	static const char * const sdi_format[] = {
		"SD",
		"HD-SDI",
		"3G-SDI",
	};
	static const char * const sdi_video[] = {
		"720x576",
		"1280x720",
		"1920x1035",
		"1920x1080",
	};
	static const char * const sdi_fps[] = {
		"undefined",
		"24p",
		"25p",
		"30p",
		"50i",
		"60i",
		"50p",
		"60p",
	};
	static const char * const chan_audio[] = {
		"HDMI",
		"SDI",
		"sgtl5000",
		"sgtl5000",
	};
	static const char * const chan_video[] = {
		"HDMI",
		"SDI",
	};

	v4l2_info(sd, "-----Chip status-----\n");

	regmap_read(state->regmap, VBX3_FPGA_REG_VERSION, &value);
	v4l2_info(sd, "FPGA version: 0x%2x\n", value);

	regmap_read(state->regmap, VBX3_FPGA_REG_GLOBAL_STATUS, &value);
	value = state->instance == 0 ? (value & 0x80) : (value & 0x40);
	v4l2_info(sd, "HDMI connected : %s\n", value ? "Yes" : "No");

	aux = state->instance == 0 ?
			VBX3_FPGA_REG_STATUS_SDI0 :
			VBX3_FPGA_REG_STATUS_SDI1;
	regmap_read(state->regmap, aux, &value);
	v4l2_info(sd, "SDI locked : %s\n", (value & 0x80) ? "Yes" : "No");
	if (value & 0x80)
		v4l2_info(sd, "SDI format %s: %s@%s\n",
				sdi_format[(value & 0x60) >> 5],
				sdi_video[(value & 0x18) >> 3],
				sdi_fps[value & 0x07]);

	v4l2_info(sd, "-----Control channels-----\n");
	aux = state->instance == 0 ?
			VBX3_FPGA_REG_CTRL_CHAN0 :
			VBX3_FPGA_REG_CTRL_CHAN1;
	regmap_read(state->regmap, aux, &value);
	v4l2_info(sd, "Channel : Audio %s, Video %s\n",
		  chan_audio[(value & 0x06)>>1],
		  chan_video[(value & 0x01)]);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vbx3_fpga_g_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	int ret;
	struct vbx_vswitch_state *state = to_state(sd);

	regmap_read(state->regmap, reg->reg, &ret);
	if (ret < 0) {
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		return ret;
	}

	reg->size = 1;
	reg->val = ret;

	return 0;
}

static int vbx3_fpga_s_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	int ret;
	struct vbx_vswitch_state *state = to_state(sd);

	ret = regmap_write(state->regmap, reg->reg, reg->val);
	if (ret < 0) {
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		return ret;
	}

	return 0;
}
#endif

/*
 * find_link_by_sinkpad_index - Return a link of an entity where the sink
 * corresponds to the same entity and the index is index
 */
static struct media_link *find_link_by_sinkpad_index(
					struct media_entity *entity,
					unsigned int index)
{
	int i = 0;
	struct media_link *result = NULL;

	while (!result && i < entity->num_links) {

		struct media_link *link = &entity->links[i];

		if (link && (link->sink->entity->id == entity->id) &&
				(link->sink->index == index))
			result = link;

		i++;
	}

	return result;
}

/*
 * vbx3_fpga_link_setup - Setup VBX3_FPGA_VSWITCH connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL or zero on success
 */
static int vbx3_fpga_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct vbx_vswitch_state *state = to_state(sd);
	struct media_link *link;
	unsigned int value, reg;

	if (!flags) {
		dev_dbg(sd->dev, "Deactivating link from %s to %s\n",
				 remote->entity->name,
				 local->entity->name);
		return 0;
	}

	switch (local->index) {
	case VBX_VSWITCH_HDMI_INPUT:

		link = find_link_by_sinkpad_index(entity,
					VBX_VSWITCH_SDI_INPUT);

		/* Check if VBX_VSWITCH_SDI_INPUT is activated */
		if (link && link->flags == MEDIA_LNK_FL_ENABLED) {

			dev_dbg(sd->dev,
				"You must first deactivate %s input\n",
				link->source->entity->name);
			return -EINVAL;
		}

		reg = state->instance == 0 ?
				VBX3_FPGA_REG_CTRL_CHAN0 :
				VBX3_FPGA_REG_CTRL_CHAN1;
		regmap_read(state->regmap, reg, &value);
		regmap_write(state->regmap,
			     reg,
			     value & 0xfe);
		break;
	case VBX_VSWITCH_SDI_INPUT:

		link = find_link_by_sinkpad_index(entity,
				VBX_VSWITCH_HDMI_INPUT);

		/* Check if VBX_VSWITCH_HDMI_INPUT is activated */
		if (link && link->flags == MEDIA_LNK_FL_ENABLED) {

			dev_dbg(sd->dev,
					"You must first deactivate %s input\n",
					link->source->entity->name);
			return -EINVAL;
		}

		reg = state->instance == 0 ?
				VBX3_FPGA_REG_CTRL_CHAN0 :
				VBX3_FPGA_REG_CTRL_CHAN1;
		regmap_read(state->regmap, reg, &value);
		regmap_write(state->regmap,
			     reg,
			     value | 0x01);
		break;
	case VBX_VSWITCH_OUTPUT:
		break;
	default:
		dev_dbg(sd->dev, "Changing to unknown pad %d\n", local->index);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops vbx_vswitch_core_ops = {
	.log_status = vbx_vswitch_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = vbx3_fpga_g_register,
	.s_register = vbx3_fpga_s_register,
#endif

};

static const struct v4l2_subdev_ops vbx_vswitch_ops = {
	.core = &vbx_vswitch_core_ops,
};

/* media operations */
static const struct media_entity_operations vbx3_fpga_media_ops = {
	.link_setup = vbx3_fpga_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static int vbx_vswitch_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct v4l2_subdev *sd;
	struct vbx_vswitch_state *state;
	int err;

	/* Now that the device is here, let's init V4L2 */
	state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!state->regmap)
		return -ENODEV;

	err = of_property_read_u32(node, "reg", &state->instance);
	if (err < 0) {
		dev_err(&pdev->dev, "Could not find instance number : %d\n", err);
		return err;
	}
	dev_dbg(&pdev->dev, "device %s : instance %u\n", node->name, state->instance);

	if (pdev->dev.of_node) {
		struct device_node *n = NULL;
		struct of_endpoint ep;

		while ((n = of_graph_get_next_endpoint(node, n))
								!= NULL) {
			err = of_graph_parse_endpoint(n, &ep);
			if (err < 0) {
				of_node_put(n);
				return err;
			}
			dev_dbg(&pdev->dev, "endpoint %d on port %d\n",
						ep.id, ep.port);
			of_node_put(n);
		}
	} else {
		dev_dbg(&pdev->dev, "No DT configuration\n");
	}
	sd = &state->sd;
	state->pdev = pdev;
	sd->owner = THIS_MODULE;
	sd->dev = &pdev->dev;
	platform_set_drvdata(pdev, &state->sd);

	v4l2_subdev_init(sd, &vbx_vswitch_ops);
	v4l2_set_subdevdata(sd, &pdev->dev);

	snprintf(sd->name, V4L2_SUBDEV_NAME_SIZE, "%s.%u",
		 node->name, state->instance);


	state->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->pads[VBX_VSWITCH_SDI_INPUT].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX_VSWITCH_HDMI_INPUT].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX_VSWITCH_OUTPUT].flags = MEDIA_PAD_FL_SOURCE;

	/* Set entity operations */
	state->sd.entity.ops = &vbx3_fpga_media_ops;

	err = media_entity_init(&sd->entity, VBX_VSWITCH_PADS_NUM, state->pads, 0);
	if (err) {
		dev_err(&pdev->dev, "Could not init entity : %d\n", err);
		return err;
	}

	err = v4l2_async_register_subdev(sd);
	if (err < 0) {
		dev_err(&pdev->dev, "Could not register async subdev : %d\n", err);
		media_entity_cleanup(&sd->entity);
		return err;
	}

	dev_dbg(&pdev->dev, "device probed\n");

	return 0;
}

static int vbx_vswitch_remove(struct platform_device *pdev)
{
/*	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);*/
	return 0;
}

static struct of_device_id vbx_vswitch_dt_id[] = {
	{ .compatible = "vbx,vswitch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vbx_vswitch_dt_id);

static struct platform_driver vbx_vswitch_pdrv = {
	.probe		= vbx_vswitch_probe,
	.remove		= vbx_vswitch_remove,
	.driver		= {
		.name	= "vbx_vswitch",
		.owner	= THIS_MODULE,
		.of_match_table	= vbx_vswitch_dt_id,
	},
};

module_platform_driver(vbx_vswitch_pdrv);
MODULE_DESCRIPTION("Video switch device driver for Veobox FPGA");
MODULE_AUTHOR("Jean-Michel Hautbois");
MODULE_LICENSE("GPL");
