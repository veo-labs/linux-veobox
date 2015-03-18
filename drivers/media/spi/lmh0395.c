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
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>

#define	LMH0395_SPI_CMD_WRITE	0x00
#define	LMH0395_SPI_CMD_READ	0x80

/* Registers of LMH0395 */
#define LMH0395_GENERAL_CTRL		0x00
#define LMH0395_OUTPUT_DRIVER		0x01
#define LMH0395_LAUNCH_AMP_CTRL		0x02
#define LMH0395_MUTE_REF		0x03
#define LMH0395_DEVICE_ID		0x04
#define	LMH0395_RATE_INDICATOR		0x05
#define	LMH0395_CABLE_LENGTH_INDICATOR	0x06
#define	LMH0395_LAUNCH_AMP_INDICATION	0x07

/* This is a one input, dual output device */
#define LMH0395_SDI_INPUT	0
#define LMH0395_SDI_OUT0	1
#define LMH0395_SDI_OUT1	2

#define LMH0395_PADS_NUM	3

#define ID_LMH0384		0x03
#define ID_LMH0394		0x13
#define ID_LMH0395		0x23

/* Register LMH0395_MUTE_REF bits [7:6] */
enum lmh0395_output_type {
	LMH0395_OUTPUT_TYPE_NONE,
	LMH0395_OUTPUT_TYPE_SDO0,
	LMH0395_OUTPUT_TYPE_SDO1,
	LMH0395_OUTPUT_TYPE_BOTH
};

static const char * const output_strs[] = {
	"No Output Driver",
	"Output Driver 0",
	"Output Driver 1",
	"Output Driver 0+1",
};


/* spi implementation */

static int lmh0395_spi_write(struct spi_device *spi, u8 reg, u8 data)
{
	int err;
	u8 cmd[2];

	cmd[0] = LMH0395_SPI_CMD_WRITE | reg;
	cmd[1] = data;

	err = spi_write(spi, cmd, 2);
	if (err < 0) {
		dev_err(&spi->dev, "SPI write failed : %d\n", err);
		return err;
	}
	dev_dbg(&spi->dev, "Wrote register %d : %02x\n", reg, data);

	return err;
}

static int lmh0395_spi_read(struct spi_device *spi, u8 reg, unsigned long *data)
{
	int err;
	u8 cmd[2];
	u8 read_data[2];

	cmd[0] = LMH0395_SPI_CMD_READ | reg;
	cmd[1] = 0xff;

	err = spi_write(spi, cmd, 2);
	if (err < 0) {
		dev_err(&spi->dev, "SPI failed to select reg : %d\n", err);
		return err;
	}

	err = spi_read(spi, read_data, 2);
	if (err < 0) {
		dev_err(&spi->dev, "SPI failed to read reg : %d\n", err);
		return err;
	}

	dev_dbg(&spi->dev, "Read register %d : [%02x,%02x]\n", reg, read_data[0], read_data[1]);
	/* The first 8 bits is the address used, drop it */
	*data = read_data[1];

	return err;
}

struct lmh0395_state {
	struct v4l2_subdev sd;
	struct media_pad pads[LMH0395_PADS_NUM];
	enum lmh0395_output_type output_type;
};

static inline struct lmh0395_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lmh0395_state, sd);
}

static int lmh0395_set_output_type(struct v4l2_subdev *sd, unsigned long output)
{
	struct lmh0395_state *state = to_state(sd);
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	unsigned long muteref_reg;

	/* Get the current register status */
	lmh0395_spi_read(spi, LMH0395_MUTE_REF, &muteref_reg);
	switch (output) {
	case LMH0395_OUTPUT_TYPE_SDO0:
		clear_bit(6, &muteref_reg);
		break;
	case LMH0395_OUTPUT_TYPE_SDO1:
		clear_bit(7, &muteref_reg);
		break;
	case LMH0395_OUTPUT_TYPE_BOTH:
		clear_bit(6, &muteref_reg);
		clear_bit(7, &muteref_reg);
		break;
	case LMH0395_OUTPUT_TYPE_NONE:
		set_bit(6, &muteref_reg);
		set_bit(7, &muteref_reg);
		break;
	default:
		return -EINVAL;
	}
	dev_dbg(&spi->dev, "Selecting %s output type\n",
					output_strs[output]);

	/* The following settings will have to be dynamic */
	set_bit(5, &muteref_reg); /* Digital Muteref */

	lmh0395_spi_write(spi, LMH0395_MUTE_REF, muteref_reg);

	state->output_type = output;
	return 0;

}

static int lmh0395_get_rate(struct v4l2_subdev *sd, u8 *rate)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	int err;
	unsigned long ctrl;

	err = lmh0395_spi_read(spi, LMH0395_RATE_INDICATOR, &ctrl);
	if (err < 0)
		return err;

	*rate = ctrl & 0x20;
	v4l2_info(sd, "Rate : %s\n", (ctrl & 0x20) ? "3G/HD" : "SD");
	return 0;
}

static int lmh0395_get_cable_length(struct v4l2_subdev *sd, u8 rate)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	u8 length;
	unsigned long cli;
	int err;

	err = lmh0395_spi_read(spi, LMH0395_CABLE_LENGTH_INDICATOR, &cli);
	if (err < 0)
		return err;

	/* The cable length indicator (CLI) provides an indication of the
	 * length of the cable attached to input. CLI is accessible via bits
	 * [7:0] of SPI register 06h.
	 * The 8-bit setting ranges in decimal value from 0 to 247
	 * ("00000000" to "11110111" binary), corresponding to 0 to 400m of
	 * Belden 1694A cable.
	 * For 3G and HD input, CLI is 1.25m per step.
	 * For SD input, CLI is 1.25m per step, less 20m, from 0 to 191 decimal
	 * and 3.5m per step from 192 to 247 decimal.
	 */

	length = cli*5/4;
	if (rate == 0) {
		if (cli <= 191)
			length -= 20;
		else
			length = ((191*5/4)-20) + ((cli-191)*7/2);

	}
	v4l2_info(sd, "Length estimated (BELDEN 1694A cables) : %dm\n",
			length);
	return 0;
}

static int lmh0395_get_control(struct v4l2_subdev *sd)
{
	int err;
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	unsigned long ctrl;
	u8 rate = 0;

	err = lmh0395_spi_read(spi, LMH0395_GENERAL_CTRL, &ctrl);
	if (err < 0)
		return err;

	if (ctrl & 0x80) {
		v4l2_info(sd, "Carrier detected\n");
		lmh0395_get_rate(sd, &rate);
		lmh0395_get_cable_length(sd, rate);
	}
	return 0;
}

static int lmh0395_get_output_status(struct v4l2_subdev *sd)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	unsigned long muteref_reg;

	/* Get the current register status */
	lmh0395_spi_read(spi, LMH0395_MUTE_REF, &muteref_reg);
	v4l2_info(sd, "Output 0 is %s\n",
			test_bit(6, &muteref_reg) ? "enabled" : "disabled");
	v4l2_info(sd, "Output 1 is %s\n",
			test_bit(7, &muteref_reg) ? "enabled" : "disabled");
	return 0;
}

static int lmh0395_log_status(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "-----Chip status-----\n");
	lmh0395_get_output_status(sd);
	lmh0395_get_control(sd);

return 0;
}

static int lmh0395_s_routing(struct v4l2_subdev *sd, u32 input, u32 output,
				u32 config)
{
	struct lmh0395_state *state = to_state(sd);

	if (state->output_type == output)
		return 0;

	return lmh0395_set_output_type(sd, output);
}

static const struct v4l2_subdev_video_ops lmh0395_video_ops = {
	.s_routing = lmh0395_s_routing,
};

static const struct v4l2_subdev_core_ops lmh0395_core_ops = {
	.log_status = lmh0395_log_status,
};

static const struct v4l2_subdev_ops lmh0395_ops = {
	.core = &lmh0395_core_ops,
	.video = &lmh0395_video_ops,
};

struct lmh0395_dev {
	unsigned long dev_id;
	char *name;
};

static const struct lmh0395_dev lmh0395_dev[] = {
	{
		.dev_id = ID_LMH0384,
		.name = "LMH0384",
	},
	{
		.dev_id = ID_LMH0394,
		.name = "LMH0394",
	},
	{
		.dev_id = ID_LMH0395,
		.name = "LMH0395",
	},
	{ /* sentinel */ },
};

static const struct spi_device_id lmh0395_id[] = {
	{ "lmh0395", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, lmh0395_id);

static const struct of_device_id lmh0395_of_match[] = {
	{.compatible = "ti,lmh0395", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, lmh0395_of_match);

static int lmh0395_probe(struct spi_device *spi)
{
	unsigned long device_id;
	struct lmh0395_state *state;
	struct v4l2_subdev *sd;
	int err, i;

	err = lmh0395_spi_read(spi, LMH0395_DEVICE_ID, &device_id);
	if (err < 0)
		return err;

	if (device_id == 0xff) {
		dev_dbg(&spi->dev, "SPI bus not ready, deferring probe\n");
		return -EPROBE_DEFER;
	}

	for (i = 0 ; i < ARRAY_SIZE(lmh0395_dev) ; i++) {
		if (device_id == lmh0395_dev[i].dev_id)
			break;
	}
	if (i == ARRAY_SIZE(lmh0395_dev)) {
		dev_err(&spi->dev, "Device not supported (id = %08lx)\n",
					device_id);
		return -ENODEV;
	}
	dev_dbg(&spi->dev, "%s detected\n", lmh0395_dev[i].name);

	/* Now that the device is here, let's init V4L2 */
	state = devm_kzalloc(&spi->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	sd = &state->sd;
	v4l2_spi_subdev_init(sd, spi, &lmh0395_ops);
	snprintf(sd->name, sizeof(sd->name), "%s-%d@spi%d",
		spi->dev.driver->name,
		spi->chip_select,
		spi->master->bus_num);

	/* Default is no output */
	lmh0395_set_output_type(sd, LMH0395_OUTPUT_TYPE_BOTH);

	if (spi->dev.of_node) {
		struct device_node *n = NULL;
		struct of_endpoint ep;

		while ((n = of_graph_get_next_endpoint(spi->dev.of_node, n))
								!= NULL) {
			err = of_graph_parse_endpoint(n, &ep);
			if (err < 0) {
				of_node_put(n);
				return err;
			}
			dev_dbg(&spi->dev, "endpoint %d on port %d\n",
						ep.id, ep.port);
			/* port 1 => SDO0 */
			if (ep.port >= 1)
				lmh0395_set_output_type(sd, ep.port);
			of_node_put(n);
		}
	} else {
		dev_dbg(&spi->dev, "No DT configuration\n");
	}

	state->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->pads[LMH0395_SDI_INPUT].flags = MEDIA_PAD_FL_SINK;
	state->pads[LMH0395_SDI_OUT0].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[LMH0395_SDI_OUT1].flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_init(&sd->entity, LMH0395_PADS_NUM, state->pads, 0);
	if (err)
		return err;

	err = v4l2_async_register_subdev(sd);
	if (err < 0) {
		media_entity_cleanup(&sd->entity);
		return err;
	}

	dev_dbg(&spi->dev, "device probed\n");

	return 0;
}

static int lmh0395_remove(struct spi_device *spi)
{
	struct v4l2_subdev *sd = spi_get_drvdata(spi);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	return 0;
}

static struct spi_driver lmh0395_driver = {
	.driver = {
		.of_match_table = lmh0395_of_match,
		.name = "lmh0395",
		.owner = THIS_MODULE,
	},
	.probe = lmh0395_probe,
	.remove = lmh0395_remove,
	.id_table = lmh0395_id,
};

module_spi_driver(lmh0395_driver);

MODULE_DESCRIPTION("spi device driver for LMH0395 equalizer");
MODULE_AUTHOR("Jean-Michel Hautbois");
MODULE_LICENSE("GPL");
