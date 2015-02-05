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

struct vbx_vswitch_state {
	struct platform_device *pdev;
	struct v4l2_subdev sd;
	struct media_pad pads[VBX_VSWITCH_PADS_NUM];
	struct regmap *regmap;
	unsigned int instance;
};

static int vbx_vswitch_log_status(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "-----Chip status-----\n");
	return 0;
}

static const struct v4l2_subdev_core_ops vbx_vswitch_core_ops = {
	.log_status = vbx_vswitch_log_status,
};

static const struct v4l2_subdev_ops vbx_vswitch_ops = {
	.core = &vbx_vswitch_core_ops,
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
