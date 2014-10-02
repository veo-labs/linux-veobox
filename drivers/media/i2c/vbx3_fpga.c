/*
 * VBX3 FPGA i2c driver.
 * Copyright (C) 2014  Jean-Michel Hautbois
 *
 * A/V source switching Vodalys VBX3
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

MODULE_DESCRIPTION("i2c device driver for VBX3 fpga source switch");
MODULE_AUTHOR("Jean-Michel Hautbois");
MODULE_LICENSE("GPL");

#define	VBX3_FPGA_REG_VERSION			0x00
#define	VBX3_FPGA_REG_CTRL_CHAN0		0x01
#define	VBX3_FPGA_REG_CTRL_CHAN1		0x02
#define	VBX3_FPGA_REG_CTRL_PATTERN_CHAN0	0x03
#define	VBX3_FPGA_REG_GLOBAL_STATUS		0x04
#define	VBX3_FPGA_REG_STATUS_SDI0		0x05
#define	VBX3_FPGA_REG_STATUS_SDI1		0x06
#define	VBX3_FPGA_REG_EVENT_CHAN0		0x07
#define	VBX3_FPGA_REG_EVENT_CHAN1		0x08

enum vbx3_fpga_input_pads {
	VBX3_FPGA_INPUT_SDI0 = 0,
	VBX3_FPGA_INPUT_ADV7611_HDMI,
	VBX3_FPGA_INPUT_SDI1,
	VBX3_FPGA_INPUT_ADV7604_HDMI,
};
enum vbx3_fpga_output_pads {
	VBX3_FPGA_OUTPUT_CHANNEL0 = 4,
	VBX3_FPGA_OUTPUT_CHANNEL1,
};

#define	VBX3_FPGA_PADS_INPUT_NUM	4
#define	VBX3_FPGA_PADS_OUTPUT_NUM	2
#define	VBX3_FPGA_PADS_NUM		VBX3_FPGA_PADS_INPUT_NUM+VBX3_FPGA_PADS_OUTPUT_NUM

struct vbx3_fpga_state {
	struct v4l2_subdev sd;
	struct media_pad pads[VBX3_FPGA_PADS_NUM];
	struct i2c_client *i2c_client;
};

static inline struct vbx3_fpga_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vbx3_fpga_state, sd);
}

static int vbx3_fpga_s_routing (struct v4l2_subdev *sd,
				u32 input, u32 output, u32 config)
{
	struct vbx3_fpga_state *state = to_state(sd);
	return 0;
}

static int vbx3_fpga_log_status(struct v4l2_subdev *sd)
{
	struct vbx3_fpga_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	u8 value;

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

	value = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_VERSION);
	v4l2_info(sd, "FPGA version: 0x%2x\n", value);

	value = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_GLOBAL_STATUS);
	v4l2_info(sd, "HDMI 0 connected : %s\n", (value & 0x80) ? "Yes" : "No");
	v4l2_info(sd, "HDMI 1 connected : %s\n", (value & 0x40) ? "Yes" : "No");
	value = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_STATUS_SDI0);
	v4l2_info(sd, "SDI 0 locked : %s\n", (value & 0x80) ? "Yes" : "No");
	if (value & 0x80)
		v4l2_info(sd, "SDI 0 format %s: %s@%s\n", sdi_format[(value & 0x60) >> 5],
						sdi_video[(value & 0x18) >> 3],
						sdi_fps[value & 0x07]);

	value = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_STATUS_SDI1);
	v4l2_info(sd, "SDI 1 locked : %s\n", (value & 0x80) ? "Yes" : "No");
	if (value & 0x80)
		v4l2_info(sd, "SDI 1 format %s: %s@%s\n", sdi_format[(value & 0x60) >> 5],
						sdi_video[(value & 0x18) >> 3],
						sdi_fps[value & 0x07]);
	v4l2_info(sd, "-----Control channels-----\n");
	value = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_CTRL_CHAN0);
	v4l2_info(sd, "Channel 0 : Audio %s, Video %s\n", chan_audio[(value & 0x06)>>1],
							  chan_video[(value & 0x01)]);
	value = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_CTRL_CHAN1);
	v4l2_info(sd, "Channel 1 : Audio %s, Video %s\n", chan_audio[(value & 0x06)>>1],
							  chan_video[(value & 0x01)]);
	return 0;
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vbx3_fpga_g_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	int ret;
	struct vbx3_fpga_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;

	ret = i2c_smbus_read_byte_data(client, reg->reg);
	if (ret < 0) {
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		return ret;
	}

	reg->size = 1;
	reg->val = ret;

	return 0;
}

static int vbx3_fpga_s_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	int ret;
	struct vbx3_fpga_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;

	ret = i2c_smbus_write_byte_data(client, reg->reg, reg->val);
	if (ret < 0) {
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		return ret;
	}

	return 0;
}
#endif

static const struct v4l2_subdev_video_ops vbx3_fpga_video_ops = {
	.s_routing = vbx3_fpga_s_routing,
};

static const struct v4l2_subdev_core_ops vbx3_fpga_core_ops = {
	.log_status = vbx3_fpga_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = vbx3_fpga_g_register,
	.s_register = vbx3_fpga_s_register,
#endif
};

static const struct v4l2_subdev_ops vbx3_fpga_ops = {
	.core = &vbx3_fpga_core_ops,
	.video = &vbx3_fpga_video_ops,
};

/* i2c implementation */

static int vbx3_fpga_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct vbx3_fpga_state *state;
	int version, status;
	int ret = 0;
	int pad;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	version = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_VERSION);
	if (version < 0) {
		v4l_err(client, "could not get version of FPGA\n");
		return -EPROBE_DEFER;
	} else {
		v4l_info(client, "version read : 0x%x\n", version);
	}
	i2c_smbus_write_byte_data(client, VBX3_FPGA_REG_CTRL_CHAN0, 0x04);
	i2c_smbus_write_byte_data(client, VBX3_FPGA_REG_CTRL_CHAN1, 0x04);
	status = i2c_smbus_read_byte_data(client, VBX3_FPGA_REG_GLOBAL_STATUS);
	v4l_info(client, "Status : 0x%x\n", status);

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;
	state->i2c_client = client;

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &vbx3_fpga_ops);
	strlcpy(sd->name, "VBX3 FPGA video switch", sizeof(sd->name));
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	state->pads[VBX3_FPGA_INPUT_SDI0].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX3_FPGA_INPUT_ADV7611_HDMI].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX3_FPGA_INPUT_SDI1].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX3_FPGA_INPUT_ADV7604_HDMI].flags = MEDIA_PAD_FL_SINK;

	state->pads[VBX3_FPGA_OUTPUT_CHANNEL0].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[VBX3_FPGA_OUTPUT_CHANNEL1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&state->sd.entity, VBX3_FPGA_PADS_NUM, state->pads, 0);
	if (ret < 0)
		v4l2_err(client, "media entity failed with error %d\n", ret);

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		media_entity_cleanup(&state->sd.entity);
		return ret;
	}

	v4l_info(client, "device probed\n");

	return ret;
}

static int vbx3_fpga_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vbx3_fpga_state *state;

	sd = &state->sd;
	media_entity_cleanup(&sd->entity);
	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id vbx3_fpga_id[] = {
	{ "vbx3_fpga", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vbx3_fpga_id);

static struct i2c_driver vbx3_fpga_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "vbx3_fpga",
	},
	.probe		= vbx3_fpga_probe,
	.remove		= vbx3_fpga_remove,
	.id_table	= vbx3_fpga_id,
};

module_i2c_driver(vbx3_fpga_driver);
