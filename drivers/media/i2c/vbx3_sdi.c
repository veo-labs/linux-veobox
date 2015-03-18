/*
 * FPGA SDI driver.
 * Copyright (C) 2015  Jean-Michel Hautbois
 *
 * Tri-Rate Serial Digital Interface PHY
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
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-of.h>

#define VBX_SDI_INPUT		0
#define VBX_SDI_OUTPUT		1
#define VBX_SDI_PADS_NUM	2

#define	VBX_SDI_REG_CTRL_PATTERN_CHAN0	0x03
#define VBX_SDI_REG_GLOBAL_STATUS	0x04
#define	VBX_SDI_REG_STATUS_SDI0		0x05
#define	VBX_SDI_REG_STATUS_SDI1		0x06
#define	VBX_SDI_REG_EVENT_CHAN0		0x07
#define	VBX_SDI_REG_EVENT_CHAN1		0x08

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

struct vbx_sdi_state {
	struct platform_device	*pdev;
	struct v4l2_subdev	sd;
	struct v4l2_dv_timings	timings;
	struct v4l2_dv_timings	detected_timings;
	struct media_pad	pads[VBX_SDI_PADS_NUM];
	struct regmap		*regmap;
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_poll_signal;
	bool			status;
	u32			polling;
	unsigned int instance;
};

struct vbx_sdi_formats {
	u32	width;
	u32	height;
	u32	fps;
	u32	interlaced;
};

/* Supported CEA timings */
static const struct v4l2_dv_timings vbx_sdi_timings[] = {
/* FIXME: is it SD format ? */
/*	V4L2_DV_BT_CEA_720X576I50,	*/
/*	V4L2_DV_BT_CEA_720X576P50,	*/
	V4L2_DV_BT_CEA_1280X720P24,
	V4L2_DV_BT_CEA_1280X720P25,
	V4L2_DV_BT_CEA_1280X720P30,
	V4L2_DV_BT_CEA_1280X720P50,
	V4L2_DV_BT_CEA_1280X720P60,
	V4L2_DV_BT_CEA_1920X1080P24,
	V4L2_DV_BT_CEA_1920X1080P25,
	V4L2_DV_BT_CEA_1920X1080P30,
	V4L2_DV_BT_CEA_1920X1080P50,
	V4L2_DV_BT_CEA_1920X1080P60,
/* FIXME: Interlaced formats not supported yet */
/*	V4L2_DV_BT_CEA_1280X720I50,	*/
/*	V4L2_DV_BT_CEA_1280X720I60,	*/
/*	V4L2_DV_BT_CEA_1920X1080I50,	*/
/*	V4L2_DV_BT_CEA_1920X1080I60,	*/
	{/* sentinel */},
};

static inline struct vbx_sdi_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vbx_sdi_state, sd);
}

static const struct vbx_sdi_formats vbx_sdi_formats[] = {
	{1280, 720, 24, 0},
	{1280, 720, 25, 0},
	{1280, 720, 30, 0},
	{1280, 720, 50, 0},
	{1280, 720, 60, 0},
/*	{1280, 720, 50, 1},
	{1280, 720, 60, 1},*/
	{1920, 1080, 24, 0},
	{1920, 1080, 25, 0},
	{1920, 1080, 30, 0},
	{1920, 1080, 50, 0},
	{1920, 1080, 60, 0},
/*	{1920, 1080, 50, 1},
	{1920, 1080, 60, 1},*/
	{/* sentinel */},
};

/* Fill the optional fields .standards and .flags in struct v4l2_dv_timings
   if the format is listed in vbx_sdi_timings[] */
static void vbx_sdi_fill_optional_dv_timings_fields(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	int i;

	for (i = 0; vbx_sdi_timings[i].bt.width; i++) {
		if (v4l2_match_dv_timings(timings, &vbx_sdi_timings[i], 250000)) {
			*timings = vbx_sdi_timings[i];
			break;
		}
	}
}

static int vbx_sdi_get_formats(struct v4l2_subdev *sd, struct vbx_sdi_formats *sdifmt)
{
	struct vbx_sdi_state *state = to_state(sd);
	u32 value;
	u32 width, height, fps, interlaced;
	int ret = 0;

	if (state->instance == 0)
		regmap_read(state->regmap, VBX_SDI_REG_STATUS_SDI0, &value);
	else
		regmap_read(state->regmap, VBX_SDI_REG_STATUS_SDI1, &value);

	switch ((value & 0x18) >> 3) {
		case 1:	width = 1280;
			height = 720;
			break;
		case 3: width = 1920;
			height = 1080;
			break;
		default: ret = -ERANGE;
			break;
	}
	switch (value & 0x7) {
		case 1: fps = 24;
			interlaced = 0;
			break;
		case 2: fps = 25;
			interlaced = 0;
			break;
		case 3: fps = 30;
			interlaced = 0;
			break;
		case 6: fps = 50;
			interlaced = 0;
			break;
		case 7: fps = 60;
			interlaced = 0;
			break;
		default: ret = -ERANGE;
			break;
	}
	sdifmt->width = width;
	sdifmt->height = height;
	sdifmt->fps = fps;
	sdifmt->interlaced = interlaced;
	return ret;
}

static int sdi2dv_timings(struct v4l2_subdev *sd, struct v4l2_dv_timings *timings)
{
	struct vbx_sdi_formats sdifmt;
	int ret = 0;
	int i;
	u32 theo_fps;
	int htot, vtot;

	if ((ret = vbx_sdi_get_formats(sd, &sdifmt) < 0))
		return ret;

	for (i = 0 ; vbx_sdi_timings[i].bt.height ; i++) {
		v4l2_dbg(2, debug, sd, "%s: %d <=> %d\n", __func__, vbx_sdi_timings[i].bt.width, sdifmt.width);
		if (vbx_sdi_timings[i].bt.width != sdifmt.width)
			continue;

		v4l2_dbg(2, debug, sd, "%s: %d <=> %d\n", __func__, vbx_sdi_timings[i].bt.height, sdifmt.height);
		if (vbx_sdi_timings[i].bt.height != sdifmt.height)
			continue;

		v4l2_dbg(2, debug, sd, "%s: %d <=> %d\n", __func__, vbx_sdi_timings[i].bt.interlaced, sdifmt.interlaced);
		if (vbx_sdi_timings[i].bt.interlaced != sdifmt.interlaced)
			continue;

		htot = V4L2_DV_BT_FRAME_WIDTH(&vbx_sdi_timings[i].bt);
		vtot = V4L2_DV_BT_FRAME_HEIGHT(&vbx_sdi_timings[i].bt);
		if (sdifmt.interlaced)
			theo_fps = 2 * sdifmt.fps * htot * vtot;
		else
			theo_fps = sdifmt.fps * htot * vtot;

		v4l2_dbg(2, debug, sd, "%s: %u <=> %llu\n", __func__, theo_fps, vbx_sdi_timings[i].bt.pixelclock);
		if (theo_fps != vbx_sdi_timings[i].bt.pixelclock)
			continue;

		*timings = vbx_sdi_timings[i];
		return 0;
	}

	v4l2_dbg(2, debug, sd, "%s: No format candidate found for %dx%d@%d%s\n", __func__,
		sdifmt.width, sdifmt.height, sdifmt.fps,
		sdifmt.interlaced ? "i" : "p");
	return -ERANGE;
}

static int vbx_sdi_log_status(struct v4l2_subdev *sd)
{
	return 0;
}

static const struct v4l2_subdev_core_ops vbx_sdi_core_ops = {
	.log_status = vbx_sdi_log_status,
};

static inline bool no_sync(struct v4l2_subdev *sd)
{
	struct vbx_sdi_state *state = to_state(sd);
	u32 value;
	bool ret = false;

	regmap_read(state->regmap, VBX_SDI_REG_GLOBAL_STATUS, &value);
	if (state->instance == 0)
		ret |= (value & 0x20);
	else
		ret |= (value & 0x08);

	return ret;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	struct vbx_sdi_state *state = to_state(sd);
	u32 value, status;

	if (state->instance == 0)
		regmap_read(state->regmap, VBX_SDI_REG_STATUS_SDI0, &value);
	else
		regmap_read(state->regmap, VBX_SDI_REG_STATUS_SDI1, &value);

	status = !(value & 0x80);

	return status;
}

static int vbx_sdi_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	*status = 0;

	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;
	*status |= no_sync(sd) ? V4L2_IN_ST_NO_SYNC : 0;

	v4l2_dbg(2, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int vbx_sdi_query_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings)
{
	int err;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_dbg(2, debug, sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	if (no_sync(sd)) {
		v4l2_dbg(2, debug, sd, "%s: signal not locked\n", __func__);
		return -ENOLINK;
	}

	err = sdi2dv_timings(sd, timings);
	if (err < 0)
		return err;

	if (debug > 1)
		v4l2_print_dv_timings(sd->name, "vbx_sdi_query_dv_timings: ",
						      timings, true);

	return 0;
}

static int vbx_sdi_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct vbx_sdi_state *state = to_state(sd);

	if (v4l2_match_dv_timings(&state->timings, timings, 0)) {
		v4l2_dbg(2, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

	vbx_sdi_fill_optional_dv_timings_fields(sd, timings);

	state->timings = *timings;

	/* TODO: Implement the call to VBX_SDI_REG_CTRL_PATTERN_CHAN0
	 * This allows to set a timing on pattern channel
	 */
	if (debug > 1)
		v4l2_print_dv_timings(sd->name, "vbx_sdi_s_dv_timings: ",
				      timings, true);
	return 0;
}

static int vbx_sdi_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct vbx_sdi_state *state = to_state(sd);

	*timings = state->timings;
	return 0;
}

static int vbx_sdi_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= 1)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_YUYV8_1X16;

	return 0;
}

static int vbx_sdi_enum_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_enum_dv_timings *timings)
{
	if (timings->index >= ARRAY_SIZE(vbx_sdi_timings) - 1)
		return -EINVAL;

	if (timings->pad >= VBX_SDI_PADS_NUM)
		return -EINVAL;

	memset(timings->reserved, 0, sizeof(timings->reserved));
	timings->timings = vbx_sdi_timings[timings->index];
	return 0;
}

static int vbx_sdi_dv_timings_cap(struct v4l2_subdev *sd,
			struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad >= VBX_SDI_PADS_NUM)
		return -EINVAL;

	cap->type = V4L2_DV_BT_656_1120;
	cap->bt.max_width = 1920;
	cap->bt.max_height = 1080;
	cap->bt.min_pixelclock = 25000000;
	cap->bt.max_pixelclock = 148500000;
	cap->bt.standards = V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			 V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT;
	cap->bt.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE |
		V4L2_DV_BT_CAP_REDUCED_BLANKING | V4L2_DV_BT_CAP_CUSTOM;
	return 0;
}

static void vbx_sdi_fill_format(struct vbx_sdi_state *state,
				struct v4l2_mbus_framefmt *format)
{
	memset(format, 0, sizeof(*format));

	format->width = state->timings.bt.width;
	format->height = state->timings.bt.height;
	/* FIXME: If we ever want to support interlaced format... */
	format->field = V4L2_FIELD_NONE;

	if (state->timings.bt.standards & V4L2_DV_BT_STD_CEA861)
		format->colorspace = (state->timings.bt.height <= 576) ?
			V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;
}


static int vbx_sdi_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct vbx_sdi_state *state = to_state(sd);

	if (format->pad >= VBX_SDI_PADS_NUM)
		return -EINVAL;

	vbx_sdi_fill_format(state, &format->format);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;
		fmt = v4l2_subdev_get_try_format(fh, format->pad);
		format->format.code = fmt->code;
	} else {
		format->format.code = MEDIA_BUS_FMT_YUYV8_1X16;
	}

	return 0;
}

static int vbx_sdi_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *format)
{
	struct vbx_sdi_state *state = to_state(sd);

	if (format->pad >= VBX_SDI_PADS_NUM)
		return -EINVAL;

	vbx_sdi_fill_format(state, &format->format);
	format->format.code = MEDIA_BUS_FMT_YUYV8_1X16;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;
		fmt = v4l2_subdev_get_try_format(fh, format->pad);
		fmt->code = format->format.code;
	} else {
		/* TODO: Fix register VBX_SDI_REG_CTRL_PATTERN_CHAN0
		 * This would allow pattern generator format to be set */
	}
	return 0;
}

static const struct v4l2_subdev_video_ops vbx_sdi_video_ops = {
	.g_input_status = vbx_sdi_g_input_status,
	.s_dv_timings = vbx_sdi_s_dv_timings,
	.g_dv_timings = vbx_sdi_g_dv_timings,
	.query_dv_timings = vbx_sdi_query_dv_timings,
};

static const struct v4l2_subdev_pad_ops vbx_sdi_pad_ops = {
	.enum_mbus_code = vbx_sdi_enum_mbus_code,
	.get_fmt = vbx_sdi_get_format,
	.set_fmt = vbx_sdi_set_format,
	.dv_timings_cap = vbx_sdi_dv_timings_cap,
	.enum_dv_timings = vbx_sdi_enum_dv_timings,
};

static const struct v4l2_subdev_ops vbx_sdi_ops = {
	.core = &vbx_sdi_core_ops,
	.video = &vbx_sdi_video_ops,
	.pad = &vbx_sdi_pad_ops,
};

static void vbx_sdi_delayed_work_poll_signal(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct vbx_sdi_state *state = container_of(dwork, struct vbx_sdi_state,
							delayed_work_poll_signal);
	struct v4l2_subdev *sd = &state->sd;
	bool status = no_signal(sd);
	struct v4l2_event events = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = 0,
	};
	struct v4l2_dv_timings	timings;
	int err = 0;

	if (state->status != status) {
		v4l2_dbg(1, debug, sd, "%s: status changed : %s -> %s\n", __func__,
			state->status ? "no signal" : "signal locked",
			status ? "no signal" : "signal locked");
		state->status = status;
		events.u.src_change.changes |= V4L2_EVENT_SRC_CH_STATUS;
	}
	err = vbx_sdi_query_dv_timings(sd, &timings);
	if (err)
		goto finish;
	if (!v4l2_match_dv_timings(&state->detected_timings, &timings, 0)) {
		v4l2_dbg(1, debug, sd, "%s: resolution changed\n", __func__);
		if (debug > 1) {
			v4l2_print_dv_timings(sd->name, "Previous format: ",
						      &state->timings, true);
			v4l2_print_dv_timings(sd->name, "Detected format: ",
							      &timings, true);
		}
		events.u.src_change.changes |= V4L2_EVENT_SRC_CH_RESOLUTION;
		state->detected_timings = timings;
	}

	if (events.u.src_change.changes)
		v4l2_subdev_notify(sd, V4L2_DEVICE_NOTIFY_EVENT, &events);

finish:
	/* Re-enable work queue */
	queue_delayed_work(state->work_queues,
		&state->delayed_work_poll_signal, state->polling);

}

static int vbx_sdi_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct v4l2_subdev *sd;
	struct vbx_sdi_state *state;
	int err;
	static const struct v4l2_dv_timings cea1280x720 = V4L2_DV_BT_CEA_1280X720P60;

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

	v4l2_subdev_init(sd, &vbx_sdi_ops);
	v4l2_set_subdevdata(sd, &pdev->dev);

	snprintf(sd->name, V4L2_SUBDEV_NAME_SIZE, "%s.%u",
		 node->name, state->instance);


	state->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->pads[VBX_SDI_INPUT].flags = MEDIA_PAD_FL_SINK;
	state->pads[VBX_SDI_OUTPUT].flags = MEDIA_PAD_FL_SOURCE;

	err = media_entity_init(&sd->entity, VBX_SDI_PADS_NUM, state->pads, 0);
	if (err) {
		dev_err(&pdev->dev, "Could not init entity : %d\n", err);
		return err;
	}

	state->timings = cea1280x720;

	/* Poll every 100 ms */
	state->polling = 100;
	state->work_queues = create_singlethread_workqueue(sd->name);
	if (!state->work_queues) {
		v4l2_err(sd, "Could not create work queue\n");
		err = -ENOMEM;
		goto err_entity;
	}
	state->status = false;

	INIT_DELAYED_WORK(&state->delayed_work_poll_signal,
		vbx_sdi_delayed_work_poll_signal);
	/* Enable work queue in 100ms */
	queue_delayed_work(state->work_queues,
		&state->delayed_work_poll_signal, state->polling);


	err = v4l2_async_register_subdev(sd);
	if (err < 0) {
		dev_err(&pdev->dev, "Could not register async subdev : %d\n", err);
		media_entity_cleanup(&sd->entity);
		goto err_workqueues;
	}

	dev_dbg(&pdev->dev, "device probed\n");

	return 0;

err_workqueues:
	cancel_delayed_work(&state->delayed_work_poll_signal);
	destroy_workqueue(state->work_queues);
err_entity:
	media_entity_cleanup(&sd->entity);
	return err;
}

static int vbx_sdi_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct vbx_sdi_state *state = to_state(sd);

	cancel_delayed_work(&state->delayed_work_poll_signal);
	destroy_workqueue(state->work_queues);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	return 0;
}

static struct of_device_id vbx_sdi_dt_id[] = {
	{ .compatible = "vbx,sdi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vbx_sdi_dt_id);

static struct platform_driver vbx_sdi_pdrv = {
	.probe		= vbx_sdi_probe,
	.remove		= vbx_sdi_remove,
	.driver		= {
		.name	= "vbx_sdi",
		.owner	= THIS_MODULE,
		.of_match_table	= vbx_sdi_dt_id,
	},
};

module_platform_driver(vbx_sdi_pdrv);
MODULE_DESCRIPTION("SDI device driver for Veobox FPGA");
MODULE_AUTHOR("Jean-Michel Hautbois");
MODULE_LICENSE("GPL");
