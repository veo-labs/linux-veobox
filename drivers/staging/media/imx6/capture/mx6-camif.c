/*
 * Video Camera Capture driver for Freescale i.MX6 SOC
 *
 * Copyright (c) 2012-2014 Mentor Graphics Inc.
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/platform_data/camera-mx6.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <media/adv7604.h>
#include <media/imx6.h>
#include <media/media-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <video/imx-ipu-v3.h>

#include "mx6-camif.h"

/*
 * Min/Max supported width and heights.
 */
#define MIN_W       176U
#define MIN_H       144U
#define MAX_W      4096U
#define MAX_H      4096U
#define MAX_W_IC   1024U
#define MAX_H_IC   1024U
#define MAX_W_VDIC  968U
#define MAX_H_VDIC 2048U

#define MX6CAM_DEF_FORMAT	V4L2_PIX_FMT_YUV420
#define MX6CAM_DEF_WIDTH	1280U
#define MX6CAM_DEF_HEIGHT	720U


#define H_ALIGN    1 /* multiple of 2 */
#define S_ALIGN    1 /* multiple of 2 */

#define DEVICE_NAME "mx6-camera"

/* In bytes, per queue */
#define VID_MEM_LIMIT	SZ_64M

static struct vb2_ops mx6cam_qops;

/*
 * The Gstreamer v4l2src plugin appears to have a bug, it doesn't handle
 * frame sizes of type V4L2_FRMSIZE_TYPE_STEPWISE correctly. Set this
 * module param to get around this bug. We can remove once v4l2src handles
 * stepwise frame sizes correctly.
 */
static int v4l2src_compat = 1;
module_param(v4l2src_compat, int, 0644);
MODULE_PARM_DESC(v4l2src_compat,
		 "Gstreamer v4l2src plugin compatibility (default: 1)");

static int v4l2src_num_buffers = 8;
module_param(v4l2src_num_buffers, int, 0644);
MODULE_PARM_DESC(v4l2src_num_buffers,
		 "Number of buffers to allocate for GStreamer (default: 8)");
static inline struct mx6cam_dev *sd2dev(struct v4l2_subdev *sd)
{
	return container_of(sd->v4l2_dev, struct mx6cam_dev, v4l2_dev);
}

static inline struct mx6cam_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct mx6cam_ctx, fh);
}

static struct mx6cam_fps_link mx6cam_fpslinks[] = {
	{
		.fps_in		= 30,
		.fps_out	= 30,
		.skip		= 0x00,
		.max_ratio	= 0,
	},
	{
		.fps_in		= 50,
		.fps_out	= 50,
		.skip		= 0x00,
		.max_ratio	= 0,
	},
	{
		.fps_in		= 50,
		.fps_out	= 30,
		.skip		= 0x05,
		.max_ratio	= 4,
	},
	{
		.fps_in		= 50,
		.fps_out	= 25,
		.skip		= 0x2,
		.max_ratio	= 1,
	},
	{
		.fps_in		= 60,
		.fps_out	= 60,
		.skip		= 0x00,
		.max_ratio	= 0,
	},
	{
		.fps_in		= 60,
		.fps_out	= 45,
		.skip		= 0x04,
		.max_ratio	= 3,
	},
	{
		.fps_in		= 60,
		.fps_out	= 30,
		.skip		= 0x02,
		.max_ratio	= 1,
	},
	{
		.fps_in		= 60,
		.fps_out	= 24,
		.skip		= 0x0b,
		.max_ratio	= 4,
	},
	{
		.fps_in		= 60,
		.fps_out	= 15,
		.skip		= 0x07,
		.max_ratio	= 3,
	},
	{
		.fps_in		= 60,
		.fps_out	= 10,
		.skip		= 0x1b,
		.max_ratio	= 5,
	},
};

/* Supported user and sensor pixel formats */
static struct mx6cam_pixfmt mx6cam_pixformats[] = {
	{
		.name	= "RGB565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.codes  = {MEDIA_BUS_FMT_RGB565_2X8_LE},
		.depth  = 16,
		.ybpp	= 2,
	}, {
		.name	= "RGB24",
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.codes  = {MEDIA_BUS_FMT_RGB888_1X24,
			   MEDIA_BUS_FMT_RGB888_2X12_LE},
		.depth  = 24,
		.ybpp	= 3,
	}, {
		.name	= "BGR24",
		.fourcc	= V4L2_PIX_FMT_BGR24,
		.depth  = 24,
		.ybpp	= 3,
	}, {
		.name	= "RGB32",
		.fourcc	= V4L2_PIX_FMT_RGB32,
		.codes = {MEDIA_BUS_FMT_ARGB8888_1X32},
		.depth  = 32,
		.ybpp	= 4,
	}, {
		.name	= "BGR32",
		.fourcc	= V4L2_PIX_FMT_BGR32,
		.depth  = 32,
		.ybpp	= 4,
	}, {
		.name	= "4:2:2 packed, YUYV",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.codes = {MEDIA_BUS_FMT_YUYV8_2X8, MEDIA_BUS_FMT_YUYV8_1X16},
		.depth  = 16,
		.ybpp	= 2,
	}, {
		.name	= "4:2:2 packed, UYVY",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.codes = {MEDIA_BUS_FMT_UYVY8_2X8, MEDIA_BUS_FMT_UYVY8_1X16},
		.depth  = 16,
		.ybpp	= 2,
	}, {
		.name	= "4:2:0 planar, YUV",
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.depth  = 12,
		.ybpp	= 1,
	}, {
		.name   = "4:2:0 planar, YVU",
		.fourcc = V4L2_PIX_FMT_YVU420,
		.depth  = 12,
		.ybpp	= 1,
	}, {
		.name   = "4:2:2 planar, YUV",
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.depth  = 16,
		.ybpp	= 1,
	}, {
		.name   = "4:2:0 planar, Y/CbCr",
		.fourcc = V4L2_PIX_FMT_NV12,
		.depth  = 12,
		.ybpp	= 1,
	}, {
		.name   = "4:2:2 planar, Y/CbCr",
		.fourcc = V4L2_PIX_FMT_NV16,
		.depth  = 16,
		.ybpp	= 1,
	},
};
#define NUM_FORMATS ARRAY_SIZE(mx6cam_pixformats)

const struct mx6cam_pixfmt *mx6cam_get_format_by_fourcc(u32 fourcc)
{
	unsigned int i;
	for ( i = 0; i < ARRAY_SIZE(mx6cam_pixformats); i++) {
		const struct mx6cam_pixfmt *format = &mx6cam_pixformats[i];

		if (format->fourcc == fourcc) {
			return format;
		}
	}

	return ERR_PTR(-EINVAL);
}

static struct mx6cam_pixfmt *mx6cam_get_format(u32 fourcc, u32 code)
{
	struct mx6cam_pixfmt *fmt, *ret = NULL;
	int i, j;

	for (i = 0; i < NUM_FORMATS; i++) {
		fmt = &mx6cam_pixformats[i];

		if (fourcc && fmt->fourcc == fourcc) {
			ret = fmt;
			goto out;
		}

		for (j = 0; fmt->codes[j]; j++) {
			if (fmt->codes[j] == code) {
				ret = fmt;
				goto out;
			}
		}
	}
out:
	return ret;
}

/* Support functions */

/* find the endpoint that is handling this input index */
#if 1
static struct mx6cam_endpoint *find_ep_by_input_index(struct mx6cam_dev *dev,
						      int input_idx)
{
	if (!dev->ep)
		return NULL;
	return (input_idx < dev->num_eps) ? &dev->eplist[input_idx] : NULL;
}
#else
static struct mx6cam_endpoint *find_ep_by_input_index(struct mx6cam_dev *dev,
						      int input_idx)
{
	struct mx6cam_endpoint *ep;
	int i;

	for (i = 0; i < dev->num_eps; i++) {
		ep = &dev->eplist[i];
		if (!ep->sd)
			continue;

		if (input_idx >= ep->sensor_input.first &&
		    input_idx <= ep->sensor_input.last)
			break;
	}

	return (i < dev->num_eps) ? ep : NULL;
}
#endif

/*
 * Search into mx6cam_fpslinks for a link with corresponding fps_in and fps_out,
 * return NULL in case it does not exist
 */
static struct mx6cam_fps_link *mx6_find_skip_link(int fps_in, int fps_out)
{
	struct mx6cam_fps_link *link;
	bool found = false;
	int i = 0;

	while (!found && i < ARRAY_SIZE(mx6cam_fpslinks)) {
		link = &mx6cam_fpslinks[i];
		found = link->fps_in == fps_in && link->fps_out == fps_out;
		i++;
	}

	link = found ? link : NULL;

	return link;
}

/*
 * Calculate the number of skip frames to get the desired fps capture set by
 * s_parm IOCTL.
 *
 * Return value will be -EINVAL if:
 * - fps src (set by dv_timings) is less < 30
 * - fps capture (set by s_parm) > fps src
 * - fps capture has not been specified by s_parm IOCTL
 * - It is not possible to make that change of fps
 */
static int mx6_calculate_skip(struct mx6cam_dev *dev)
{
	int ret = 0;
	struct mx6cam_fps_link *link = NULL;
	int fps_in = dev->in_parm.parm.capture.timeperframe.denominator /
			dev->in_parm.parm.capture.timeperframe.numerator;
	int fps_out = dev->out_parm.parm.capture.timeperframe.denominator /
			dev->out_parm.parm.capture.timeperframe.numerator;

	if (!dev->out_parm.parm.capture.capability || fps_in < 30 ||
		!fps_out || fps_out > fps_in)
		return -EINVAL;

	dev_dbg(dev->v4l2_dev.dev, "Calculating skip for fpsin %d, psout %d\n",
			fps_in,
			fps_out);

	link = mx6_find_skip_link(fps_in, fps_out);

	if (!link && (fps_in % 2 == 0)) {

		/* We try to reduce by half of fps_in */
		dev_dbg(dev->v4l2_dev.dev,
			"Not a perfect match, reducing fps to %d\n",
			fps_in / 2);
		link = mx6_find_skip_link(fps_in, fps_in / 2);
	}

	if (link) {
		dev->max_ratio = link->max_ratio;
		dev->skip = link->skip;
	} else {
		dev->max_ratio = 0;
		dev->skip = 0;
		dev_dbg(dev->v4l2_dev.dev,
			"Change of fps not supported\n");
		ret = -EINVAL;
	}

	return ret;
}

static void update_format_from_timings(struct mx6cam_dev *dev, struct v4l2_dv_timings *timings)
{
	int fps, hsize, vsize;

	dev->format.pixelformat = V4L2_PIX_FMT_YUYV;
	dev->format.width = timings->bt.width;
	dev->format.height = timings->bt.height;
	if (timings->bt.interlaced) {
		dev->format.field = V4L2_FIELD_ALTERNATE;
		dev->format.height /= 2;
	} else {
		dev->format.field = V4L2_FIELD_NONE;
	}
	dev->format.colorspace = V4L2_COLORSPACE_REC709;
	/*
	 * The YUYV format is four bytes for every two pixels, so bytesperline
	 * is width * 2.
	 */
	dev->format.bytesperline = dev->format.width * 2;
	dev->format.sizeimage = dev->format.bytesperline * dev->format.height;
	dev->format.priv = 0;
	dev->user_pixfmt = mx6cam_get_format_by_fourcc(dev->format.pixelformat);

	/* Update subdev_fmt as well */
	dev->subdev_fmt.width = dev->format.width;
	dev->subdev_fmt.height = dev->format.height;
	dev->subdev_fmt.colorspace = V4L2_COLORSPACE_REC709;
	dev->subdev_fmt.code = MEDIA_BUS_FMT_YUYV8_1X16;
	dev->subdev_pixfmt = mx6cam_get_format(0, dev->subdev_fmt.code);

	/* Update fps from timings */
	hsize = timings->bt.hfrontporch + timings->bt.hsync +
			timings->bt.hbackporch + timings->bt.width;
	vsize = timings->bt.vfrontporch + timings->bt.vsync +
			timings->bt.vbackporch + timings->bt.il_vfrontporch +
			timings->bt.il_vsync + timings->bt.il_vbackporch +
			timings->bt.height;
	fps = (unsigned)timings->bt.pixelclock / (hsize * vsize);

	/* Update in/out parm */
	dev->in_parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	dev->in_parm.parm.capture.timeperframe.numerator = 1;
	dev->in_parm.parm.capture.timeperframe.denominator = fps;

	if (dev->out_parm.parm.capture.capability) {
		dev_dbg(dev->v4l2_dev.dev,
			"Recalculating skip parameters for %dfps\n",
			fps);
		mx6_calculate_skip(dev);
	}
}

/*
 * Query sensor and update signal lock status. Returns true if lock
 * status has changed.
 */
static bool update_signal_lock_status(struct mx6cam_dev *dev)
{
	bool locked, changed;
	u32 status;
	int ret;

	ret = v4l2_subdev_call(dev->ep->sd, video, g_input_status, &status);
	if (ret)
		return false;

	locked = ((status & V4L2_IN_ST_NO_SYNC) == 0);
	changed = (dev->signal_locked != locked);
	dev->signal_locked = locked;
	v4l2_err(&dev->v4l2_dev,
		"status: %d, locked: %d, changed: %d, dev->signal_locked: %d\n",
		status, locked, changed, dev->signal_locked);

	return changed;
}

/*
 * Return true if the VDIC deinterlacer is needed. We need the VDIC
 * if the sensor is transmitting fields, and userland is requesting
 * motion compensation (rather than simple weaving).
 */
static bool need_vdic(struct mx6cam_dev *dev,
		      struct v4l2_mbus_framefmt *sf)
{
	return dev->motion != MOTION_NONE && V4L2_FIELD_HAS_BOTH(sf->field);
}

/*
 * Return true if sensor format currently meets the VDIC
 * restrictions:
 *     o the full-frame resolution to the VDIC must be at or below 968x2048.
 *     o the pixel format to the VDIC must be YUV422
 */
static bool can_use_vdic(struct mx6cam_dev *dev,
			 struct v4l2_mbus_framefmt *sf)
{
	return sf->width <= MAX_W_VDIC &&
		sf->height <= MAX_H_VDIC &&
		(sf->code == MEDIA_BUS_FMT_UYVY8_2X8 ||
		 sf->code == MEDIA_BUS_FMT_UYVY8_1X16 ||
		 sf->code == MEDIA_BUS_FMT_YUYV8_2X8 ||
		 sf->code == MEDIA_BUS_FMT_YUYV8_1X16);
}

/*
 * Return true if the current capture parameters require the use of
 * the Image Converter. We need the IC for scaling, colorspace conversion,
 * preview, and rotation.
 */
static bool need_ic(struct mx6cam_dev *dev,
		    struct v4l2_mbus_framefmt *sf,
		    struct v4l2_pix_format *uf,
		    struct v4l2_rect *crop)
{
	enum ipu_color_space sensor_cs, user_cs;
	bool ret;

	sensor_cs = ipu_mbus_code_to_colorspace(sf->code);
	user_cs = ipu_pixelformat_to_colorspace(uf->pixelformat);
#if 0
	ret = (uf->width != crop->width ||
	       uf->height != crop->height ||
	       user_cs != sensor_cs ||
	       dev->preview_on ||
	       dev->rot_mode != IPU_ROTATE_NONE);
#else
	ret = false;
#endif
	if (!ret)
		v4l2_err(&dev->v4l2_dev, "No need for IC\n");
	else {
		v4l2_err(&dev->v4l2_dev, "IC needed :\n");
		v4l2_err(&dev->v4l2_dev, "Crop [%dx%d] -> User [%dx%d]\n",
					crop->width, crop->height, uf->width, uf->height);
		if (user_cs != sensor_cs)
			v4l2_err(&dev->v4l2_dev, "CS: %d != %d\n", user_cs, sensor_cs);
		if (dev->rot_mode != IPU_ROTATE_NONE)
			v4l2_err(&dev->v4l2_dev, "Rotation mode is %d\n", dev->rot_mode);
		if (dev->preview_on)
			v4l2_err(&dev->v4l2_dev, "Preview is active\n");
	}

	return ret;
}

/*
 * Return true if user and sensor formats currently meet the IC
 * restrictions:
 *     o the parallel CSI bus cannot be 16-bit wide.
 *     o the endpoint id must be 0 (for MIPI CSI2, the endpoint id is the
 *       virtual channel number, and only VC0 can pass through the IC).
 *     o the resizer output size must be at or below 1024x1024.
 */
static bool can_use_ic(struct mx6cam_dev *dev,
		       struct v4l2_mbus_framefmt *sf,
		       struct v4l2_pix_format *uf)
{
	struct mx6cam_endpoint *ep = dev->ep;
	bool ret;

#if 0
	ret = (ep->ep.bus_type == V4L2_MBUS_CSI2 ||
		ep->ep.bus.parallel.bus_width < 16) &&
		ep->ep.base.id == 0 &&
		uf->width <= MAX_W_IC &&
		uf->height <= MAX_H_IC;

		if (!ret) {
			v4l2_err(&dev->v4l2_dev, "IC can't be used :\n");
			if (ep->ep.bus_type != V4L2_MBUS_CSI2)
				v4l2_err(&dev->v4l2_dev, "bus type is not CSI2 : %d != %d\n", ep->ep.bus_type, V4L2_MBUS_CSI2);
			if (ep->ep.bus.parallel.bus_width >= 16)
				v4l2_err(&dev->v4l2_dev, "bus width : %d > 16\n", ep->ep.bus.parallel.bus_width);
			if (ep->ep.base.id != 0)
				v4l2_err(&dev->v4l2_dev, "base id : %d != 0", ep->ep.base.id);
			if (uf->width > MAX_W_IC)
				v4l2_err(&dev->v4l2_dev, "width %d > %d", uf->width, MAX_W_IC);
			if (uf->height > MAX_H_IC)
				v4l2_err(&dev->v4l2_dev, "height %d > %d", uf->height, MAX_H_IC);
	} else {
		v4l2_err(&dev->v4l2_dev, "IC is usuable\n");
	}
#else
	ret = false;
	v4l2_err(&dev->v4l2_dev, "IC can't be used :\n");
#endif
	return ret;
}

/*
 * Adjusts passed width and height to meet IC resizer limits.
 */
static void adjust_to_resizer_limits(struct mx6cam_dev *dev,
				     struct v4l2_pix_format *uf,
				     struct v4l2_rect *crop)
{
	u32 *width, *height;

	if (dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		width = &uf->width;
		height = &uf->height;
	} else {
		width = &uf->width;
		height = &uf->height;
	}

	/* output of resizer can't be above 1024x1024 */
	*width = min_t(__u32, *width, MAX_W_IC);
	*height = min_t(__u32, *height, MAX_H_IC);

	/* resizer cannot downsize more than 8:1 */
	if (dev->rot_mode >= IPU_ROTATE_90_RIGHT) {
		*height = max_t(__u32, *height, crop->width / 8);
		*width = max_t(__u32, *width, crop->height / 8);
	} else {
		*width = max_t(__u32, *width, crop->width / 8);
		*height = max_t(__u32, *height, crop->height / 8);
	}
}

static void adjust_user_fmt(struct mx6cam_dev *dev,
			    struct v4l2_mbus_framefmt *sf,
			    struct v4l2_pix_format *uf,
			    struct v4l2_rect *crop)
{
	struct mx6cam_pixfmt *fmt;

	/*
	 * Make sure resolution is within IC resizer limits
	 * if we need the Image Converter.
	 */
	if (need_ic(dev, sf, uf, crop))
		adjust_to_resizer_limits(dev, uf, crop);

	/*
	 * Force the resolution to match crop window if
	 * we can't use the Image Converter.
	 */
	if (!can_use_ic(dev, sf, uf)) {
		uf->width = crop->width;
		uf->height = crop->height;
	}

	fmt = mx6cam_get_format(uf->pixelformat, 0);

	uf->bytesperline = uf->width * fmt->ybpp;
	uf->sizeimage = (uf->width * uf->height * fmt->depth) / 8;
}

/*
 * Calculate what the default active crop window should be. Ask
 * the sensor via g_crop. This crop window will be stored to dev->crop.
 */
static void calc_default_crop(struct mx6cam_dev *dev,
			      struct v4l2_rect *rect,
			      struct v4l2_mbus_framefmt *sf)
{
	struct v4l2_crop crop;
	int ret;

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = v4l2_subdev_call(dev->ep->sd, video, g_crop, &crop);
	if (ret) {
		/* sensor doesn't support .g_crop(), assume sensor frame */
		rect->top = rect->left = 0;
		if (((sf->width >= MIN_W) && (sf->width <= MAX_W))
			&& ((sf->height >= MIN_H) && (sf->height <= MAX_H))) {
			rect->width = sf->width;
			rect->height = sf->height;
			v4l2_err(&dev->v4l2_dev, "Sensor does not support cropping, applying [%dx%d]\n", rect->width, rect->height);
		}
	} else
		*rect = crop.c;

	/* adjust crop window to h/w alignment restrictions */
	rect->width &= ~0x7;
	rect->left &= ~0x3;
}

static int update_sensor_std(struct mx6cam_dev *dev)
{
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Update STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}
	v4l2_err(&dev->v4l2_dev, "Update STD on input %s\n", epinput->name[sensor_input]);

	return v4l2_subdev_call(dev->ep->sd, video, querystd,
				&dev->current_std);
}

static int update_subdev_fmt(struct mx6cam_dev *dev)
{
	struct mx6cam_endpoint *ep;
	struct v4l2_subdev_format sd_fmt;
	int ret;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

#if 0
	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS) {
		ret = v4l2_subdev_call(dev->ep->sd, video, g_mbus_fmt,
				       &dev->subdev_fmt);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(dev->ep->sd, video, g_mbus_config,
				       &dev->mbus_cfg);
		if (ret)
			return ret;

	} else {
		/* TODO: This should be dynamic */
		sd_fmt.pad = 1;
		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(dev->ep->sd, pad, get_fmt, NULL, &sd_fmt);
		if (ret)
			return ret;
		dev->subdev_fmt = sd_fmt.format;
	}
#else
	/* TODO: This should be dynamic */
	if (strcmp(dev->ep->sd->name, "adv7611 1-004c") == 0)
		sd_fmt.pad = 1;
	else
		sd_fmt.pad = 6;

	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(dev->ep->sd, pad, get_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;
	dev->subdev_fmt = sd_fmt.format;
#endif
	dev->subdev_pixfmt = mx6cam_get_format(0, dev->subdev_fmt.code);

	/* update sensor crop bounds */
	dev->crop_bounds.top = dev->crop_bounds.left = 0;
	dev->crop_bounds.width = dev->subdev_fmt.width;
	dev->crop_bounds.height = dev->subdev_fmt.height;
	dev->crop_defrect = dev->crop_bounds;

	dev->crop.top = 0;
	dev->crop.width = dev->subdev_fmt.width;
	dev->crop.height = dev->subdev_fmt.height;
	/* For the moment, fill mbus_cfg in order to force type
	 * This should be done in a clean way
	 */
	dev->mbus_cfg.type = V4L2_MBUS_BT656;

	return 0;
}

/*
 * Turn current sensor power on/off according to power_count.
 */
#if 1
static int sensor_set_power(struct mx6cam_dev *dev, int on)
{
	return 0;
}
#else
static int sensor_set_power(struct mx6cam_dev *dev, int on)
{
	struct mx6cam_endpoint *ep = dev->ep;
	struct v4l2_subdev *sd = ep->sd;
	int ret;

	if (on && ep->power_count++ > 0)
		return 0;
	else if (!on && (ep->power_count == 0 || --ep->power_count > 0))
		return 0;

	ret = v4l2_subdev_call(sd, core, s_power, on);
	return ret != -ENOIOCTLCMD ? ret : 0;
}
#endif

/*
 * Turn current sensor streaming on/off according to stream_count.
 */
static int sensor_set_stream(struct mx6cam_dev *dev, int on)
{
	struct mx6cam_endpoint *ep = dev->ep;
	struct v4l2_subdev *sd = ep->sd;
	int ret;

	if (on && ep->stream_count++ > 0)
		return 0;
	else if (!on && (ep->stream_count == 0 || --ep->stream_count > 0))
		return 0;

	ret = v4l2_subdev_call(sd, video, s_stream, on);
	return ret != -ENOIOCTLCMD ? ret : 0;
}

/*
 * Start the encoder for buffer streaming. There must be at least two
 * frames in the vb2 queue.
 */
static int start_encoder(struct mx6cam_dev *dev)
{
	struct v4l2_subdev *streaming_sd;
	int ret;

	if (dev->encoder_on)
		return 0;

	/* sensor stream on */
	ret = sensor_set_stream(dev, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "sensor stream on failed\n");
		return ret;
	}

	/* encoder/vdic stream on */
	streaming_sd = dev->using_vdic ? dev->vdic_sd : dev->encoder_sd;

	ret = v4l2_subdev_call(streaming_sd, video, s_stream, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "encoder stream on failed\n");
		return ret;
	}

	dev->encoder_on = true;
	return 0;
}

/*
 * Stop the encoder.
 */
static int stop_encoder(struct mx6cam_dev *dev)
{
	struct v4l2_subdev *streaming_sd;
	int ret;

	if (!dev->encoder_on)
		return 0;

	streaming_sd = dev->using_vdic ? dev->vdic_sd : dev->encoder_sd;

	/* encoder/vdic off */
	ret = v4l2_subdev_call(streaming_sd, video, s_stream, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "encoder stream off failed\n");

	/* sensor stream off */
	ret = sensor_set_stream(dev, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "sensor stream off failed\n");

	dev->encoder_on = false;
	return ret;
}

/*
 * Start preview.
 */
static int start_preview(struct mx6cam_dev *dev)
{
	int ret;

	if (atomic_read(&dev->status_change)) {
		update_signal_lock_status(dev);
		update_sensor_std(dev);
		update_subdev_fmt(dev);
		/* reset active crop window */
		calc_default_crop(dev, &dev->crop, &dev->subdev_fmt);
		atomic_set(&dev->status_change, 0);
		v4l2_info(&dev->v4l2_dev, "at preview on: %s, %s\n",
			  v4l2_norm_to_name(dev->current_std),
			  dev->signal_locked ? "signal locked" : "no signal");
	}

	/* sensor stream on */
	ret = sensor_set_stream(dev, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "sensor stream on failed\n");
		return ret;
	}

	/* preview stream on */
	ret = v4l2_subdev_call(dev->preview_sd, video, s_stream, 1);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "preview stream on failed\n");

	return ret;
}

/*
 * Stop preview.
 */
static int stop_preview(struct mx6cam_dev *dev)
{
	int ret;

	/* preview stream off */
	ret = v4l2_subdev_call(dev->preview_sd, video, s_stream, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "preview stream off failed\n");
		return ret;
	}

	/* sensor stream off */
	ret = sensor_set_stream(dev, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "sensor stream off failed\n");

	return ret;
}

/*
 * Start/Stop streaming.
 */
static int set_stream(struct mx6cam_dev *dev, bool on)
{
	int ret = 0;

	if (on) {
		dev_dbg(dev->dev, "start streaming\n");
		if (atomic_read(&dev->status_change)) {
			update_signal_lock_status(dev);
			update_sensor_std(dev);
			update_subdev_fmt(dev);
			/* reset active crop window */
			calc_default_crop(dev, &dev->crop, &dev->subdev_fmt);
			atomic_set(&dev->status_change, 0);
			v4l2_info(&dev->v4l2_dev, "at stream on: %s, %s\n",
				  v4l2_norm_to_name(dev->current_std),
				  dev->signal_locked ?
				  "signal locked" : "no signal");
		}

		dev->using_ic =
			(need_ic(dev, &dev->subdev_fmt, &dev->format,
				 &dev->crop) &&
			 can_use_ic(dev, &dev->subdev_fmt, &dev->format));

		dev->using_vdic = need_vdic(dev, &dev->subdev_fmt) &&
			can_use_vdic(dev, &dev->subdev_fmt);

		if (dev->preview_on)
			stop_preview(dev);

		ret = start_encoder(dev);

		if (dev->preview_on)
			start_preview(dev);
	} else {
		dev_dbg(dev->dev, "stop streaming\n");
		ret = stop_encoder(dev);
	}

	return ret;
}

/*
 * Restart work handler. This is called in three cases during active
 * streaming and/or preview:
 *
 * o NFB4EOF errors
 * o A decoder's signal lock status or autodetected video standard changes
 * o End-of-Frame timeouts
 */
static void restart_work_handler(struct work_struct *w)
{
	struct mx6cam_dev *dev = container_of(w, struct mx6cam_dev,
					      restart_work);

	mutex_lock(&dev->mutex);

	if (!vb2_start_streaming_called(&dev->buffer_queue)) {
		/* just restart preview if on */
		if (dev->preview_on) {
			v4l2_warn(&dev->v4l2_dev, "restarting preview\n");
			stop_preview(dev);
			start_preview(dev);
		}
		goto out_unlock;
	}

	v4l2_warn(&dev->v4l2_dev, "restarting\n");

	set_stream(dev, false);
	set_stream(dev, true);

out_unlock:
	mutex_unlock(&dev->mutex);
}

/*
 * Restart timer function. Schedules a restart.
 */
static void mx6cam_restart_timeout(unsigned long data)
{
	struct mx6cam_dev *dev = (struct mx6cam_dev *)data;

	schedule_work(&dev->restart_work);
}

/* Controls */
static int mx6cam_set_rotation(struct mx6cam_dev *dev,
			       int rotation, bool hflip, bool vflip)
{
	enum ipu_rotate_mode rot_mode;
	int ret;

	ret = ipu_degrees_to_rot_mode(&rot_mode, rotation,
				      hflip, vflip);
	if (ret)
		return ret;

	if (rot_mode != dev->rot_mode) {
		/* can't change rotation mid-streaming */
		if (vb2_is_streaming(&dev->buffer_queue)) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed while streaming\n",
				 __func__);
			return -EBUSY;
		}

		if (rot_mode != IPU_ROTATE_NONE &&
		    !can_use_ic(dev, &dev->subdev_fmt, &dev->format)) {
			v4l2_err(&dev->v4l2_dev,
				"%s: current format does not allow rotation\n",
				 __func__);
			return -EINVAL;
		}
	}

	dev->rot_mode = rot_mode;
	dev->rotation = rotation;
	dev->hflip = hflip;
	dev->vflip = vflip;

	return 0;
}

static int mx6cam_set_motion(struct mx6cam_dev *dev,
			     enum ipu_motion_sel motion)
{
	if (motion != dev->motion) {
		/* can't change motion setting mid-streaming */
		if (vb2_is_streaming(&dev->buffer_queue)) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed while streaming\n",
				 __func__);
			return -EBUSY;
		}

		if (motion != MOTION_NONE && dev->preview_on) {
			v4l2_err(&dev->v4l2_dev,
				 "Preview is on, cannot enable deinterlace\n");
			return -EBUSY;
		}

		if (motion != MOTION_NONE &&
		    !can_use_vdic(dev, &dev->subdev_fmt)) {
			v4l2_err(&dev->v4l2_dev,
				 "sensor format does not allow deinterlace\n");
			return -EINVAL;
		}
	}

	dev->motion = motion;
	return 0;
}

static int mx6cam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mx6cam_dev *dev = container_of(ctrl->handler,
					      struct mx6cam_dev, ctrl_hdlr);
	enum ipu_motion_sel motion;
	bool hflip, vflip;
	int rotation;

	rotation = dev->rotation;
	hflip = dev->hflip;
	vflip = dev->vflip;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			hflip = (ctrl->val == 1);
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			vflip = (ctrl->val == 1);
		break;
	case V4L2_CID_ROTATE:
		if (ctrl->val)
			rotation = ctrl->val;
		break;
	case V4L2_CID_IMX6_MOTION:
		if (ctrl->val)
			motion = ctrl->val;
		return mx6cam_set_motion(dev, motion);
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid control\n");
		return -ERANGE;
	}

	return mx6cam_set_rotation(dev, rotation, hflip, vflip);
}

static const struct v4l2_ctrl_ops mx6cam_ctrl_ops = {
	.s_ctrl = mx6cam_s_ctrl,
};

static const struct v4l2_ctrl_config motion_cfg = {
	.ops = &mx6cam_ctrl_ops,
	.id = V4L2_CID_IMX6_MOTION,
	.name = "Motion Compensation",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = MOTION_NONE,
	.min = MOTION_NONE,
	.max = HIGH_MOTION,
	.step = 1,
};

static int mx6cam_init_controls(struct mx6cam_dev *dev)
{
	struct v4l2_ctrl_handler *hdlr = &dev->ctrl_hdlr;
	int ret;

	v4l2_ctrl_handler_init(hdlr, 4);

	v4l2_ctrl_new_std(hdlr, &mx6cam_ctrl_ops, V4L2_CID_HFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdlr, &mx6cam_ctrl_ops, V4L2_CID_VFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdlr, &mx6cam_ctrl_ops, V4L2_CID_ROTATE,
			  0, 270, 90, 0);
	v4l2_ctrl_new_custom(hdlr, &motion_cfg, NULL);

	if (hdlr->error) {
		ret = hdlr->error;
		v4l2_ctrl_handler_free(hdlr);
		return ret;
	}

	dev->v4l2_dev.ctrl_handler = hdlr;
	dev->vfd.ctrl_handler = hdlr;

	v4l2_ctrl_handler_setup(hdlr);

	return 0;
}


/*
 * Video ioctls follow
 */

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strlcpy(cap->driver, DEVICE_NAME, sizeof(cap->driver));
	strlcpy(cap->card, DEVICE_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:mx6-camera", sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
//			| V4L2_CAP_VIDEO_OVERLAY;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct mx6cam_pixfmt *fmt;

	if (f->index >= NUM_FORMATS)
		return -EINVAL;

	fmt = &mx6cam_pixformats[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

#if 0
static int vidioc_enum_fmt_vid_overlay(struct file *file, void *priv,
				       struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt_vid_cap(file, priv, f);
}
#endif
static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	f->fmt.pix = dev->format;

	return 0;
}
#if 0
static int vidioc_g_fmt_vid_overlay(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	f->fmt.win = dev->win;
	return 0;
}
#endif
#if 1
static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	const struct mx6cam_pixfmt *info;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	/* Retrieve format information and select the default format if the
	 * requested format isn't supported.
	 */
	info = mx6cam_get_format_by_fourcc(pix->pixelformat);
	if (IS_ERR(info))
		info = mx6cam_get_format_by_fourcc(MX6CAM_DEF_FORMAT);

	pix->pixelformat = info->fourcc;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->field = V4L2_FIELD_NONE;
	pix->width = clamp(pix->width, MIN_W, MAX_W);
	pix->height = clamp(pix->height, MIN_H, MAX_H);
	pix->bytesperline = pix->width * info->ybpp;
	pix->sizeimage = (pix->width * pix->height * info->depth) / 8;

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *format)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	vidioc_try_fmt_vid_cap(file, priv, format);

	if (vb2_is_busy(&dev->buffer_queue)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	dev->format = format->fmt.pix;
	dev->user_pixfmt = mx6cam_get_format_by_fourcc(dev->format.pixelformat);

	return 0;
}

#else
static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct mx6cam_pixfmt *fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format sd_fmt;
	unsigned int width_align;
	struct v4l2_rect crop;
	int ret;

	/*
	 * We have to adjust the width such that the physaddrs and U and
	 * U and V plane offsets are multiples of 8 bytes as required by
	 * the IPU DMA Controller. For the planar formats, this corresponds
	 * to a pixel alignment of 16. For all the packed formats, 8 is
	 * good enough.
	 */
	width_align = ipu_pixelformat_is_planar(fmt->fourcc) ? 4 : 3;

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W,
			      width_align, &f->fmt.pix.height,
			      MIN_H, MAX_H, H_ALIGN, S_ALIGN);

	v4l2_fill_mbus_format(&mbus_fmt, &f->fmt.pix, 0);

	ret = v4l2_subdev_call(dev->ep->sd, video, try_mbus_fmt, &mbus_fmt);
	if (ret)
		return ret;

	fmt = mx6cam_get_format(0, mbus_fmt.code);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Sensor mbus format (0x%08x) invalid.\n",
			 mbus_fmt.code);
		return -EINVAL;
	}
	/*
	 * calculate what the optimal crop window will be for this
	 * sensor format and make any user format adjustments.
	 */
	calc_default_crop(dev, &crop, &sd_fmt.format);
	adjust_user_fmt(dev, &sd_fmt.format, f, &crop);

	fmt = mx6cam_get_format(0, sd_fmt.format.code);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	pix->pixelformat = fmt->fourcc;

	pix->bytesperline = (pix->width * fmt->depth) >> 3;
	pix->sizeimage = pix->height * pix->bytesperline;
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	return 0;
}

static int vidioc_try_fmt_vid_overlay(struct file *file, void *priv,
				      struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_window *win = &f->fmt.win;
	unsigned int width_align;

	width_align = ipu_pixelformat_is_planar(dev->fbuf.fmt.pixelformat) ?
		4 : 3;

	v4l_bound_align_image(&win->w.width, MIN_W, MAX_W_IC,
			      width_align, &win->w.height,
			      MIN_H, MAX_H_IC, H_ALIGN, S_ALIGN);

	adjust_to_resizer_limits(dev, &f->fmt.pix, &dev->crop);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format sd_fmt;
	int ret;

	if (vb2_is_busy(&dev->buffer_queue)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	v4l2_fill_mbus_format(&mbus_fmt, &f->fmt.pix, 0);
	ret = v4l2_subdev_call(dev->ep->sd, video, s_mbus_fmt, &mbus_fmt);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "%s s_mbus_fmt failed\n", __func__);
		return ret;
	}
	ret = update_subdev_fmt(dev);
	if (ret)
		return ret;

	/* reset active crop window */
	calc_default_crop(dev, &dev->crop, &dev->subdev_fmt);

	return 0;
}
#endif

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_pixfmt *fmt;
	struct v4l2_pix_format uf;

	fmt = mx6cam_get_format(fsize->pixel_format, 0);
	if (!fmt)
		return -EINVAL;

	if (v4l2src_compat)
		return v4l2_subdev_call(dev->ep->sd, video,
					enum_framesizes, fsize);

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = MIN_W;
	fsize->stepwise.step_width =
		ipu_pixelformat_is_planar(fmt->fourcc) ? 16 : 8;
	fsize->stepwise.min_height = MIN_H;
	fsize->stepwise.step_height = 1 << H_ALIGN;

	uf = dev->format;
	uf.pixelformat = fmt->fourcc;

	if (need_ic(dev, &dev->subdev_fmt, &uf, &dev->crop)) {
		fsize->stepwise.max_width = MAX_W_IC;
		fsize->stepwise.max_height = MAX_H_IC;
	} else {
		fsize->stepwise.max_width = MAX_W;
		fsize->stepwise.max_height = MAX_H;
	}

	return 0;
}

static int vidioc_enum_frameintervals(struct file *file, void *priv,
				      struct v4l2_frmivalenum *fival)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_pixfmt *fmt;

	fmt = mx6cam_get_format(fival->pixel_format, 0);
	if (!fmt)
		return -EINVAL;

	return v4l2_subdev_call(dev->ep->sd, video, enum_frameintervals, fival);
}
#if 0
static int vidioc_s_fmt_vid_overlay(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_window *win = &f->fmt.win;
	int ret;

	ret = vidioc_try_fmt_vid_overlay(file, priv, f);
	if (ret)
		return ret;

	if (dev->preview_on)
		stop_preview(dev);

	dev->win = *win;

	if (dev->preview_on)
		start_preview(dev);

	return 0;
}
#endif

static int vidioc_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Query STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}

	v4l2_err(&dev->v4l2_dev, "Query STD on input %s\n", epinput->name[sensor_input]);
	return update_sensor_std(dev);
}

static int vidioc_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int ret;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Get STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}

	v4l2_err(&dev->v4l2_dev, "Get STD on input %s\n", epinput->name[sensor_input]);
	ret = v4l2_subdev_call(dev->ep->sd, video, g_std, std);
	if (ret < 0)
		return ret;

	*std = dev->current_std;
	return 0;
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int ret;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Set STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}
	v4l2_err(&dev->v4l2_dev, "Set STD on input %s\n", epinput->name[sensor_input]);

	if (vb2_is_busy(&dev->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(dev->ep->sd, video, s_std, std);
	if (ret < 0)
		return ret;

	dev->current_std = std;
	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,
			     struct v4l2_input *input)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_endpoint *ep;

	/* find the endpoint that is handling this input */
	ep = find_ep_by_input_index(dev, input->index);
	if (!ep)
		return -EINVAL;

	strncpy(input->name, ep->sd->name, sizeof(ep->sd->name));

	input->type = V4L2_INPUT_TYPE_CAMERA;
	/* TODO: Modify it to make it dynamic */
	input->capabilities = V4L2_IN_CAP_DV_TIMINGS;

	if (input->index == dev->current_input) {
		v4l2_subdev_call(ep->sd, video, g_input_status, &input->status);
/*		update_sensor_std(dev);
		input->std = dev->current_std;*/
	} else {
		input->status = V4L2_IN_ST_NO_SIGNAL;
		input->std = V4L2_STD_UNKNOWN;
	}

	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	*index = dev->current_input;
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int index)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_endpoint *ep;
	int ret;

	if (index == dev->current_input)
		return 0;

	/* find the endpoint that is handling this input */
	ep = find_ep_by_input_index(dev, index);
	if (!ep)
		return -EINVAL;

	if (dev->ep != ep) {
		/*
		 * don't allow switching sensors if there are queued buffers,
		 * preview is on, or there are other users of the current
		 * sensor besides us.
		 */
		if (vb2_is_busy(&dev->buffer_queue) || dev->preview_on ||
		    dev->ep->power_count > 1)
			return -EBUSY;

		v4l2_info(&dev->v4l2_dev, "switching to sensor %s\n",
			  ep->sd->name);

		/* power down current sensor before enabling new one */
		ret = sensor_set_power(dev, 0);
		if (ret)
			v4l2_warn(&dev->v4l2_dev, "sensor power off failed\n");

		/* set new endpoint */
		dev->ep = ep;

		/* power-on the new sensor */
		ret = sensor_set_power(dev, 1);
		if (ret)
			v4l2_warn(&dev->v4l2_dev, "sensor power on failed\n");

		/* power-on the csi2 receiver */
		if (dev->ep->ep.bus_type == V4L2_MBUS_CSI2 && dev->csi2_sd) {
			ret = v4l2_subdev_call(dev->csi2_sd, core, s_power,
					       true);
			if (ret)
				v4l2_err(&dev->v4l2_dev,
					 "csi2 power on failed\n");
			else
				v4l2_err(&dev->v4l2_dev, "CSI2 is powered\n");
		}/* else {
			if (ep->ep.bus_type != V4L2_MBUS_CSI2)
				v4l2_err(&dev->v4l2_dev, "bus type is not CSI2 : %d != %d\n", ep->ep.bus_type, V4L2_MBUS_CSI2);
		}*/
	}

	dev->vfd.tvnorms = 0;
/*
 * TODO: Make it dynamic
 *	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS)
 *		dev->vfd->tvnorms = V4L2_STD_ALL;
 */
	ret = v4l2_subdev_call(dev->ep->sd, video, s_routing, 0, 0, 0);

	dev->current_input = index;

	/*
	 * sometimes on switching video input on video decoder devices
	 * no lock status change event is generated, but vertical sync
	 * is messed up nevertheless. So schedule a restart to correct it.
	 */
	if (ctx->io_allowed)
		mod_timer(&dev->restart_timer,
			  jiffies + msecs_to_jiffies(MX6CAM_RESTART_DELAY));

	return 0;
}

static int vidioc_g_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *a)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	int ret;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	dev_dbg(dev->v4l2_dev.dev, "Calling to g_parm of subdev %s\n",
		 dev->ep->sd->name);
	ret = v4l2_subdev_call(dev->ep->sd, video, g_parm, a);

	if (ret) {

		/* FPS has not been set yet */
		if (dev->out_parm.parm.capture.capability == 0) {
			memcpy(&dev->out_parm, &dev->in_parm,
				sizeof(struct v4l2_streamparm));
		}

		memcpy(a, &dev->out_parm, sizeof(struct v4l2_streamparm));
		ret = 0;
	}
	return ret;
}

static int vidioc_s_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *a)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	int ret;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	dev_dbg(dev->v4l2_dev.dev, "Calling to s_parm of subdev %s\n",
			 dev->ep->sd->name);
	ret = v4l2_subdev_call(dev->ep->sd, video, s_parm, a);

	if (!ret)
		return 0;

	/* Update out_parm */
	memcpy(&dev->out_parm, a, sizeof(struct v4l2_captureparm));
	dev->out_parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	ret = mx6_calculate_skip(dev);

	if (ret)
		dev->out_parm.parm.capture.capability = 0;

	return ret;
}

static int vidioc_cropcap(struct file *file, void *priv,
			  struct v4l2_cropcap *cropcap)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    cropcap->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	cropcap->bounds = dev->crop_bounds;
	cropcap->defrect = dev->crop_defrect;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *priv,
			 struct v4l2_crop *crop)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    crop->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	crop->c = dev->crop;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *priv,
			 const struct v4l2_crop *crop)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_rect *bounds = &dev->crop_bounds;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    crop->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	if (vb2_is_busy(&dev->buffer_queue))
		return -EBUSY;

	/* make sure crop window is within bounds */
	if (crop->c.top < 0 || crop->c.left < 0 ||
	    crop->c.left + crop->c.width > bounds->width ||
	    crop->c.top + crop->c.height > bounds->height)
		return -EINVAL;

	/*
	 * FIXME: the IPU currently does not setup the CCIR code
	 * registers properly to handle arbitrary vertical crop
	 * windows. So return error if the sensor bus is BT.656
	 * and user is asking to change vertical cropping.
	 */
	if (dev->ep->ep.bus_type == V4L2_MBUS_BT656 &&
	    (crop->c.top != dev->crop.top ||
	     crop->c.height != dev->crop.height)) {
		v4l2_err(&dev->v4l2_dev,
			 "vertical crop is not supported for this sensor!\n");
		return -EINVAL;
	}

	dev->crop = crop->c;

	/* adjust crop window to h/w alignment restrictions */
	dev->crop.width &= ~0x7;
	dev->crop.left &= ~0x3;

	/*
	 * Crop window has changed, we need to adjust the user
	 * width/height to meet new IC resizer restrictions or to
	 * match the new crop window if the IC can't be used.
	 */
	adjust_user_fmt(dev, &dev->subdev_fmt, &dev->format,
			&dev->crop);

	return 0;
}

#if 0
static int vidioc_s_fbuf(struct file *file, void *priv,
			 const struct v4l2_framebuffer *fbuf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_pixfmt *fmt;

	if (fbuf->flags != V4L2_FBUF_FLAG_OVERLAY || !fbuf->base)
		return -EINVAL;

	fmt = mx6cam_get_format(fbuf->fmt.pixelformat, 0);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 fbuf->fmt.pixelformat);
		return -EINVAL;
	}

	if (dev->preview_on)
		stop_preview(dev);

	dev->fbuf = *fbuf;
	dev->fbuf_pixfmt = fmt;

	if (dev->preview_on)
		start_preview(dev);

	return 0;
}

static int vidioc_g_fbuf(struct file *file, void *priv,
			 struct v4l2_framebuffer *fbuf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	*fbuf = dev->fbuf;

	return 0;
}

static int vidioc_overlay(struct file *file, void *priv,
			  unsigned int enable)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	int ret = 0;

	if (!ctx->io_allowed)
		return -EBUSY;

	if (enable && !dev->preview_on) {
		if (vb2_is_streaming(&dev->buffer_queue) && !dev->using_ic) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed while streaming w/o IC\n",
				 __func__);
			return -EBUSY;
		}

		if (dev->motion != MOTION_NONE) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed with deinterlacing\n",
				 __func__);
			return -EBUSY;
		}

		if (!can_use_ic(dev, &dev->subdev_fmt, &dev->format)) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: current format does not allow preview\n",
				 __func__);
			return -EINVAL;
		}

		ret = start_preview(dev);
		if (!ret)
			dev->preview_on = true;
	} else if (!enable && dev->preview_on) {
		ret = stop_preview(dev);
		dev->preview_on = false;
	}

	return ret;
}
#endif

static int vidioc_log_status(struct file *file, void *f)
{
	struct video_device *vdev = video_devdata(file);

	v4l2_ctrl_log_status(file, f);
	v4l2_device_call_all(vdev->v4l2_dev, 0, core, log_status);
	return 0;
}

static int vidioc_s_edid(struct file *file, void *f, struct v4l2_edid *edid)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	return v4l2_subdev_call(dev->ep->sd, pad, set_edid, edid);
}

static int vidioc_g_edid(struct file *file, void *f, struct v4l2_edid *edid)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	return v4l2_subdev_call(dev->ep->sd, pad, get_edid, edid);
}

static int vidioc_g_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_endpoint *ep;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	return v4l2_subdev_call(dev->ep->sd,
			video, g_dv_timings, timings);
}

static int vidioc_s_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_endpoint *ep;
	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	ret = v4l2_subdev_call(dev->ep->sd,
			video, s_dv_timings, timings);

	dev->dv_timings_cap = *timings;
	update_format_from_timings(dev, timings);

	sd_fmt.format.code = dev->subdev_fmt.code;
	if (strcmp(dev->ep->sd->name, "adv7604 1-0020") == 0)
		sd_fmt.pad = 6;
	else
		sd_fmt.pad = 1;

	ret = v4l2_subdev_call(dev->ep->sd, pad, set_fmt, NULL, &sd_fmt);
	return ret;

}

static int vidioc_query_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_endpoint *ep;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	return v4l2_subdev_call(dev->ep->sd,
			video, query_dv_timings, timings);
}
static int vidioc_enum_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_enum_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_endpoint *ep;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	timings->pad = 0;
	v4l2_err(&dev->v4l2_dev, "Enum DV Timings on input %s\n", ep->sd->name);
	return v4l2_subdev_call(dev->ep->sd,
			pad, enum_dv_timings, timings);
}

static int vidioc_dv_timings_cap(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings_cap *cap)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	cap->pad = 0;
	return v4l2_subdev_call(dev->ep->sd,
			pad, dv_timings_cap, cap);
}
#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vidioc_g_register(struct file *file, void *f,
				struct v4l2_dbg_register *reg)
{
	struct video_device *vdev = video_devdata(file);
	v4l2_device_call_all(vdev->v4l2_dev, 0, core, g_register, reg);
	return 0;
}
static int vidioc_s_register(struct file *file, void *f,
				const struct v4l2_dbg_register *reg)
{
	struct video_device *vdev = video_devdata(file);
	v4l2_device_call_all(vdev->v4l2_dev, 0, core, s_register, reg);
	return 0;
}
#endif

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
		case V4L2_EVENT_CTRL:
			return v4l2_ctrl_subscribe_event(fh, sub);
		case V4L2_EVENT_SOURCE_CHANGE:
			return v4l2_src_change_event_subscribe(fh, sub);
		default:
			break;
	}
	return -EINVAL;
};

static const struct v4l2_ioctl_ops mx6cam_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
	.vidioc_enum_fmt_vid_cap        = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap           = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap         = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = vidioc_s_fmt_vid_cap,

	.vidioc_enum_framesizes         = vidioc_enum_framesizes,
	.vidioc_enum_frameintervals     = vidioc_enum_frameintervals,

/*	.vidioc_enum_fmt_vid_overlay    = vidioc_enum_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay	= vidioc_g_fmt_vid_overlay,
	.vidioc_try_fmt_vid_overlay	= vidioc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay	= vidioc_s_fmt_vid_overlay,*/

	.vidioc_querystd		= vidioc_querystd,
	.vidioc_g_std           = vidioc_g_std,
	.vidioc_s_std           = vidioc_s_std,

	.vidioc_enum_input      = vidioc_enum_input,
	.vidioc_g_input         = vidioc_g_input,
	.vidioc_s_input         = vidioc_s_input,

	.vidioc_g_parm          = vidioc_g_parm,
	.vidioc_s_parm          = vidioc_s_parm,

/*	.vidioc_g_fbuf          = vidioc_g_fbuf,
	.vidioc_s_fbuf          = vidioc_s_fbuf,*/

	.vidioc_cropcap         = vidioc_cropcap,
	.vidioc_g_crop          = vidioc_g_crop,
	.vidioc_s_crop          = vidioc_s_crop,


	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf	= vb2_ioctl_prepare_buf,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,
	.vidioc_expbuf		= vb2_ioctl_expbuf,
	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,

/*	.vidioc_overlay         = vidioc_overlay,*/

	.vidioc_log_status      = vidioc_log_status,
	.vidioc_s_edid          = vidioc_s_edid,
	.vidioc_g_edid          = vidioc_g_edid,
	.vidioc_s_dv_timings	= vidioc_s_dv_timings,
	.vidioc_g_dv_timings	= vidioc_g_dv_timings,
	.vidioc_query_dv_timings = vidioc_query_dv_timings,
	.vidioc_enum_dv_timings	= vidioc_enum_dv_timings,
	.vidioc_dv_timings_cap	= vidioc_dv_timings_cap,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register      = vidioc_g_register,
	.vidioc_s_register	= vidioc_s_register,
#endif
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};


/*
 * Queue operations
 */

/*
 * Setup the constraints of the queue
 */
static int mx6cam_queue_setup(struct vb2_queue *vq,
			      const struct v4l2_format *fmt,
			      unsigned int *nbuffers, unsigned int *nplanes,
			      unsigned int sizes[], void *alloc_ctxs[])
{
	struct mx6cam_dev *dev = vb2_get_drv_priv(vq);

	if (vq->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Check sufficient buffer numbers have been allocated
	 * 4 is a good minimum
	 */
	if (vq->num_buffers + *nbuffers < v4l2src_num_buffers)
		*nbuffers = v4l2src_num_buffers - vq->num_buffers;

	if (fmt && fmt->fmt.pix.sizeimage < dev->format.sizeimage)
		return -EINVAL;

	*nplanes = 1;
	sizes[0] = fmt ? fmt->fmt.pix.sizeimage : dev->format.sizeimage;
	alloc_ctxs[0] = dev->alloc_ctx;

	dprintk(dev, "get %d buffer(s) of size %d each.\n",
			*nbuffers, sizes[0]);
	return 0;
}

/*
 * Prepare the buffer for queueing to the DMA engine
 */
static int mx6cam_buf_prepare(struct vb2_buffer *vb)
{
	struct mx6cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = dev->format.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dprintk(dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

/*
 * Queue this buffer to the DMA engine
 */
static void mx6cam_buf_queue(struct vb2_buffer *vb)
{
	struct mx6cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct mx6cam_buffer *buf = to_mx6cam_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	list_add_tail(&buf->list, &dev->buf_list);
	dev_dbg(dev->dev, "buffer %d added to list\n", buf->vb.v4l2_buf.index);
	spin_unlock_irqrestore(&dev->irqlock, flags);

}

static int mx6cam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mx6cam_dev *dev = vb2_get_drv_priv(vq);
	struct mx6cam_buffer *frame;
	unsigned long flags;
	int ret;

	dev->sequence = 0;

	ret = set_stream(dev, true);
	if (ret) {
		spin_lock_irqsave(&dev->irqlock, flags);
		/* release all active buffers */
		while (!list_empty(&dev->buf_list)) {
			frame = list_entry(dev->buf_list.next,
					   struct mx6cam_buffer, list);
			list_del(&frame->list);
			vb2_buffer_done(&frame->vb, VB2_BUF_STATE_QUEUED);
			dev_dbg(dev->dev, "buffer %d done\n", frame->vb.v4l2_buf.index);
		}
		spin_unlock_irqrestore(&dev->irqlock, flags);
	}
	dev_dbg(dev->dev, "streaming started\n");
	return ret;
}

static void mx6cam_stop_streaming(struct vb2_queue *vq)
{
	struct mx6cam_dev *dev = vb2_get_drv_priv(vq);
	struct mx6cam_buffer *buf, *nbuf;
	unsigned long flags;

	set_stream(dev, false);

	spin_lock_irqsave(&dev->irqlock, flags);

	list_for_each_entry_safe(buf, nbuf, &dev->buf_list, list) {
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		list_del(&buf->list);
		dev_dbg(dev->dev, "buffer %d done\n", buf->vb.v4l2_buf.index);
	}

	spin_unlock_irqrestore(&dev->irqlock, flags);
}

static struct vb2_ops mx6cam_qops = {
	.queue_setup	 = mx6cam_queue_setup,
	.buf_prepare	 = mx6cam_buf_prepare,
	.buf_queue	 = mx6cam_buf_queue,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
	.start_streaming = mx6cam_start_streaming,
	.stop_streaming  = mx6cam_stop_streaming,
};

/*
 * File operations
 */
static int mx6cam_open(struct file *file)
{
	struct mx6cam_dev *dev = video_drvdata(file);
	struct mx6cam_ctx *ctx;
	struct mx6cam_graph_entity *entity;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	if (!dev->ep) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	entity = list_first_entry(&dev->entities, struct mx6cam_graph_entity, list);
	if (!entity) {
		v4l2_err(&dev->v4l2_dev, "no entity registered\n");
		ret = -ENODEV;
		goto unlock;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->dev = dev;
	v4l2_fh_add(&ctx->fh);
	ctx->io_allowed = false;

	file->private_data = &ctx->fh;
	ret = sensor_set_power(dev, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "sensor power on failed\n");
		goto ctx_free;
	}

	if (dev->ep->ep.bus_type == V4L2_MBUS_CSI2 && dev->csi2_sd) {
		ret = v4l2_subdev_call(dev->csi2_sd, core, s_power, true);
		if (ret) {
			v4l2_err(&dev->v4l2_dev, "csi2 power on failed\n");
			goto sensor_off;
		} else
			v4l2_err(&dev->v4l2_dev, "CSI2 is powered\n");
	}/* else {
		if (dev->ep->ep.bus_type != V4L2_MBUS_CSI2)
			v4l2_err(&dev->v4l2_dev, "bus type is not CSI2 : %d != %d\n", dev->ep->ep.bus_type, V4L2_MBUS_CSI2);
	}*/

	/* update the sensor's current format */
	update_subdev_fmt(dev);
	/* and init crop window if needed */
	if (!dev->crop.width || !dev->crop.height)
		calc_default_crop(dev, &dev->crop, &dev->subdev_fmt);

	mutex_unlock(&dev->mutex);
	return 0;

sensor_off:
	sensor_set_power(dev, 0);
ctx_free:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
unlock:
	mutex_unlock(&dev->mutex);
	return ret;
}

static int mx6cam_release(struct file *file)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	int ret = 0;

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_lock(&dev->mutex);

	if (ctx->io_allowed) {
		BUG_ON(dev->io_ctx != ctx);

		vb2_queue_release(&dev->buffer_queue);
		/* We are no longer owner of the queue */

		if (dev->preview_on) {
			stop_preview(dev);
			dev->preview_on = false;
		}

		dev->io_ctx = NULL;
	}

	ret = sensor_set_power(dev, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "sensor power off failed\n");

	mutex_unlock(&dev->mutex);
	kfree(ctx);
	return ret;
}

#if 0
static unsigned int mx6cam_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct vb2_queue *vq = &dev->buffer_queue;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	ret = vb2_poll(vq, file, wait);

	mutex_unlock(&dev->mutex);
	return ret;
}

static int mx6cam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct vb2_queue *vq = &dev->buffer_queue;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	ret = vb2_mmap(vq, vma);

	mutex_unlock(&dev->mutex);
	return ret;
}
#endif

static const struct v4l2_file_operations mx6cam_fops = {
	.owner		= THIS_MODULE,
	.open		= mx6cam_open,
	.release	= vb2_fop_release, /*mx6cam_release,*/
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

static void mx6cam_send_source_change(struct mx6cam_dev *dev, struct v4l2_event *ev)
{
	unsigned i;

	for (i = 0; i < dev->num_eps; i++) {
		ev->id = i;
		v4l2_event_queue(&dev->vfd, ev);
	}
}
/*
 * Handle notifications from the subdevs.
 */
static void mx6cam_subdev_notification(struct v4l2_subdev *sd,
				       unsigned int notification,
				       void *arg)
{
	struct mx6cam_dev *dev;

	if (sd == NULL)
		return;

	dev = sd2dev(sd);

	switch (notification) {
	case MX6CAM_NFB4EOF_NOTIFY:
		if (dev)
			mod_timer(&dev->restart_timer, jiffies +
				  msecs_to_jiffies(MX6CAM_RESTART_DELAY));
		break;
	case DECODER_STATUS_CHANGE_NOTIFY:
		atomic_set(&dev->status_change, 1);
		if (dev) {
			v4l2_warn(&dev->v4l2_dev, "decoder status change\n");
			mod_timer(&dev->restart_timer, jiffies +
				  msecs_to_jiffies(MX6CAM_RESTART_DELAY));
		}
		break;
	case MX6CAM_EOF_TIMEOUT_NOTIFY:
		if (dev) {
			/* cancel a running restart timer since we are
			   restarting now anyway */
			del_timer_sync(&dev->restart_timer);
			/* and restart now */
			schedule_work(&dev->restart_work);
		}
		break;
	case V4L2_DEVICE_NOTIFY_EVENT:
		if (dev) {
			atomic_set(&dev->status_change, 1);
			mx6cam_send_source_change(dev, (struct v4l2_event *)arg);
			v4l2_warn(&dev->v4l2_dev, "Format change on %s\n", sd->name);
			break;
		}
	}
}

#if 0
static int mx6cam_add_sensor(struct mx6cam_dev *dev,
			     struct device_node *remote,
			     struct mx6cam_endpoint *ep)
{
	struct platform_device *sensor_pdev = NULL;
	struct i2c_client *i2c_client;
	struct device *sensor_dev;
	int ret = 0;

	i2c_client = of_find_i2c_device_by_node(remote);
	if (!i2c_client)
		sensor_pdev = of_find_device_by_node(remote);

	if (!i2c_client && !sensor_pdev)
		return -EPROBE_DEFER;

	sensor_dev = i2c_client ? &i2c_client->dev : &sensor_pdev->dev;

	device_lock(sensor_dev);

	if (!sensor_dev->driver ||
	    !try_module_get(sensor_dev->driver->owner)) {
		ret = -EPROBE_DEFER;
		v4l2_info(&dev->v4l2_dev, "No driver found for %s\n",
			  remote->full_name);
		goto unlock;
	}

	ep->sd = i2c_client ? i2c_get_clientdata(i2c_client) :
		dev_get_drvdata(sensor_dev);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, ep->sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed to register subdev %s\n",
			 ep->sd->name);
		goto mod_put;
	}

	v4l2_info(&dev->v4l2_dev, "Registered sensor subdev %s on CSI%d\n",
		  ep->sd->name, ep->ep.base.port);
	ret = 0;

mod_put:
	module_put(sensor_dev->driver->owner);
unlock:
	device_unlock(sensor_dev);
	put_device(sensor_dev);
	return ret;
}
#endif

static int mx6cam_add_csi2_receiver(struct mx6cam_dev *dev)
{
	struct platform_device *pdev;
	struct device_node *node;
	int ret = -EPROBE_DEFER;

	node = of_find_compatible_node(NULL, NULL, "fsl,imx6-mipi-csi2");
	if (!node)
		return 0;

	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return 0;

	/* Lock to ensure dev->driver won't change. */
	device_lock(&pdev->dev);

	if (!pdev->dev.driver || !try_module_get(pdev->dev.driver->owner))
		goto dev_unlock;

	dev->csi2_sd = dev_get_drvdata(&pdev->dev);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->csi2_sd);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to register mipi_csi2!\n");
		goto mod_put;
	}

	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->csi2_sd->name);
mod_put:
	module_put(pdev->dev.driver->owner);
dev_unlock:
	device_unlock(&pdev->dev);
	if (ret == -EPROBE_DEFER)
		v4l2_info(&dev->v4l2_dev,
			  "deferring mipi_csi2 registration\n");
	else if (ret < 0)
		v4l2_err(&dev->v4l2_dev,
			 "mipi_csi2 registration failed (%d)\n", ret);
	return ret;
}

#if 0
/* parse inputs property from v4l2_of_endpoint node */
static int mx6cam_parse_inputs(struct mx6cam_dev *dev,
			       struct device_node *node,
			       int next_input,
			       struct mx6cam_endpoint *ep)
{
	struct mx6cam_sensor_input *epinput = &ep->sensor_input;
	int ret, i;

	for (i = 0; i < MX6CAM_MAX_INPUTS; i++) {
		const char *input_name;
		u32 val;

		ret = of_property_read_u32_index(node, "inputs", i, &val);
		if (ret)
			break;

		epinput->value[i] = val;

		ret = of_property_read_string_index(node, "input-names", i,
						    &input_name);
		if (!ret)
			strncpy(epinput->name[i], input_name,
				sizeof(epinput->name[i]));
		else
			snprintf(epinput->name[i], sizeof(epinput->name[i]),
				 "%s-%d", ep->sd->name, i);

		val = 0;
		ret = of_property_read_u32_index(node, "input-caps", i, &val);
		epinput->caps[i] = val;

	}

	epinput->num = i;

	/* if no inputs provided just assume a single input */
	if (epinput->num == 0) {
		epinput->num = 1;
		epinput->caps[0] = 0;
		strncpy(epinput->name[0], ep->sd->name,
			sizeof(epinput->name[0]));
	}

	epinput->first = next_input;
	epinput->last = next_input + epinput->num - 1;
	return epinput->last + 1;
}

static int mx6cam_parse_endpoints(struct mx6cam_dev *dev,
				  struct device_node *node)
{
	struct device_node *remote, *epnode = NULL;
	struct v4l2_of_endpoint ep;
	int ret, next_input = 0;

	while (dev->num_eps < MX6CAM_MAX_ENDPOINTS) {
		epnode = of_graph_get_next_endpoint(node, epnode);
		if (!epnode)
			break;

		v4l2_of_parse_endpoint(epnode, &ep);

		if (ep.base.port > 1) {
			v4l2_err(&dev->v4l2_dev, "invalid port %d\n",
				 ep.base.port);
			of_node_put(epnode);
			return -EINVAL;
		}

		remote = of_graph_get_remote_port_parent(epnode);
		if (!remote) {
			v4l2_err(&dev->v4l2_dev,
				 "failed to find remote port parent\n");
			of_node_put(epnode);
			return -EINVAL;
		}

		dev->eplist[dev->num_eps].ep = ep;
#if 0
		ret = mx6cam_add_sensor(dev, remote,
					&dev->eplist[dev->num_eps]);
		if (ret)
			goto out;
#endif

		next_input = mx6cam_parse_inputs(dev, epnode, next_input,
						 &dev->eplist[dev->num_eps]);

		dev->num_eps++;

		of_node_put(remote);
		of_node_put(epnode);
	}

	if (!dev->num_eps) {
		v4l2_err(&dev->v4l2_dev, "no endpoints defined!\n");
		return -EINVAL;
	}

	dev->ep = &dev->eplist[0];
	return 0;

out:
	of_node_put(remote);
	of_node_put(epnode);
	return ret;
}
#endif

static void mx6cam_unregister_subdevs(struct mx6cam_dev *dev)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct mx6cam_endpoint *ep;
	int i;

	if (!IS_ERR_OR_NULL(dev->encoder_sd))
		v4l2_device_unregister_subdev(dev->encoder_sd);

	if (!IS_ERR_OR_NULL(dev->preview_sd))
		v4l2_device_unregister_subdev(dev->preview_sd);

	if (!IS_ERR_OR_NULL(dev->vdic_sd))
		v4l2_device_unregister_subdev(dev->vdic_sd);

	if (!IS_ERR_OR_NULL(dev->csi2_sd))
		v4l2_device_unregister_subdev(dev->csi2_sd);

	for (i = 0; i < dev->num_eps; i++) {
		ep = &dev->eplist[i];
		if (!ep->sd)
			continue;
		client = v4l2_get_subdevdata(ep->sd);
		if (!client)
			continue;

		v4l2_device_unregister_subdev(ep->sd);

		if (!client->dev.of_node) {
			adapter = client->adapter;
			i2c_unregister_device(client);
			if (adapter)
				i2c_put_adapter(adapter);
		}
	}
}


static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct ipu_soc *mx6cam_find_ipu(struct mx6cam_dev *dev,
				       struct device_node *node)
{
	struct device_node *ipu_node;
	struct device *ipu_dev;

	ipu_node = of_parse_phandle(node, "ipu", 0);
	if (!ipu_node) {
		v4l2_err(&dev->v4l2_dev, "missing ipu phandle!\n");
		return NULL;
	}

	ipu_dev = bus_find_device(&platform_bus_type, NULL,
				  ipu_node, of_dev_node_match);
	of_node_put(ipu_node);

	if (!ipu_dev) {
		v4l2_err(&dev->v4l2_dev, "failed to find ipu device!\n");
		return NULL;
	}

	return dev_get_drvdata(ipu_dev);
}

static struct mx6cam_graph_entity *
mx6cam_graph_find_entity(struct mx6cam_dev *camdev,
			const struct device_node *node)
{
	struct mx6cam_graph_entity *entity;

	list_for_each_entry(entity, &camdev->entities, list) {
		if (entity->node == node) {
			return entity;
		}
	}

	return NULL;
}

static int mx6cam_graph_build_one(struct mx6cam_dev *camdev,
				struct mx6cam_graph_entity *entity)
{
	u32 link_flags = 0;
	struct media_entity *local = entity->entity;
	struct media_entity *remote;
	struct media_pad *local_pad;
	struct media_pad *remote_pad;
	struct mx6cam_graph_entity *ent;
	struct v4l2_of_link link;
	struct device_node *ep = NULL;
	struct device_node *next;
	int ret = 0;

	dev_dbg(camdev->dev, "creating links for entity %s\n", local->name);

	while (1) {
		/* Get the next endpoint and parse its link. */
		next = of_graph_get_next_endpoint(entity->node, ep);
		if (next == NULL)
			break;

		of_node_put(ep);
		ep = next;

		//dev_dbg(camdev->dev, "processing endpoint %s\n", ep->full_name);

		ret = v4l2_of_parse_link(ep, &link);
		if (ret < 0) {
			dev_err(camdev->dev, "failed to parse link for %s\n",
				ep->full_name);
			continue;
		}

		/* Skip sink ports, they will be processed from the other end of
		 * the link.
		 */
		if (link.local_port >= local->num_pads) {
			dev_err(camdev->dev, "invalid local port number "
								"%u on %s\n",
				link.local_port, link.local_node->full_name);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		local_pad = &local->pads[link.local_port];

		if (local_pad->flags & MEDIA_PAD_FL_SINK) {
			dev_dbg(camdev->dev, "skipping sink port %s:%u\n",
				link.local_node->full_name, link.local_port);
			v4l2_of_put_link(&link);
			continue;
		}

		/* Find the remote entity. */
		ent = mx6cam_graph_find_entity(camdev, link.remote_node);
		if (ent == NULL) {
			dev_err(camdev->dev, "no entity found for %s\n",
				link.remote_node->full_name);
			v4l2_of_put_link(&link);
			ret = -ENODEV;
			break;
		}

		remote = ent->entity;

		if (link.remote_port >= remote->num_pads) {
			dev_err(camdev->dev, "invalid remote port number "
								"%u on %s\n",
				link.remote_port, link.remote_node->full_name);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		remote_pad = &remote->pads[link.remote_port];

		v4l2_of_put_link(&link);

		/* Create the media link. */
		dev_dbg(camdev->dev, "creating %s:%u -> %s:%u link\n",
			local->name, local_pad->index,
			remote->name, remote_pad->index);

		ret = media_entity_create_link(local, local_pad->index,
						remote, remote_pad->index,
						link_flags);
		if (ret < 0) {
			dev_err(camdev->dev, "failed to create %s:%u -> %s:%u link\n",
				local->name, local_pad->index,
				remote->name, remote_pad->index);
			break;
		}
	}

	of_node_put(ep);
	return ret;
}

static int mx6cam_probe_complete(struct mx6cam_dev *dev)
{
	struct platform_device *pdev = container_of(dev->dev, struct platform_device, dev);
	struct video_device *vfd;
	struct pinctrl *pinctrl;
	int ret;

	/* Create device node */
	vfd = &dev->vfd;
	strlcpy(vfd->name, "mx6-camera", sizeof(vfd->name));
	vfd->fops = &mx6cam_fops;
	vfd->ioctl_ops = &mx6cam_ioctl_ops;
	vfd->release = video_device_release_empty;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->queue = &dev->buffer_queue;

	/*
	 * Provide a mutex to v4l2 core. It will be used to protect
	 * all fops and v4l2 ioctls.
	 */
	vfd->lock = &dev->mutex;
	dev->v4l2_dev.notify = mx6cam_subdev_notification;

	snprintf(vfd->name, sizeof(vfd->name), "%s", dev->dev->of_node->name);

	/* Get any pins needed */
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	/* setup some defaults */
	dev->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	atomic_set(&dev->status_change, 1);
	dev->signal_locked = 0;
	dev->in_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->out_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->out_parm.parm.capture.capability = 0; /* Still not configure */
	dev->skip = 0;
	dev->max_ratio = 0;

	/* Initialize the buffer queues */
	vfd->queue = &dev->buffer_queue;
	dev->alloc_ctx = vb2_dma_contig_init_ctx(dev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "Failed to alloc vb2 context\n");
		goto unreg_vdev;
	}

	dev->buffer_queue.type = dev->type;
	dev->buffer_queue.io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	dev->buffer_queue.lock = &dev->mutex;
	dev->buffer_queue.drv_priv = dev;
	dev->buffer_queue.buf_struct_size = sizeof(struct mx6cam_buffer);
	dev->buffer_queue.ops = &mx6cam_qops;
	dev->buffer_queue.mem_ops = &vb2_dma_contig_memops;
	dev->buffer_queue.min_buffers_needed = 2;
	dev->buffer_queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
			| V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	ret = vb2_queue_init(&dev->buffer_queue);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "Failed to init vb2 queue: %d\n", ret);
		goto cleanup_ctx;
	}

	INIT_LIST_HEAD(&dev->buf_list);
	INIT_WORK(&dev->restart_work, restart_work_handler);
	init_timer(&dev->restart_timer);
	dev->restart_timer.data = (unsigned long)dev;
	dev->restart_timer.function = mx6cam_restart_timeout;

	/* init our controls */
	ret = mx6cam_init_controls(dev);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "init controls failed\n");
		goto unreg_vdev;
	}

	video_set_drvdata(vfd, dev);
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto unreg_vdev;
	}

	platform_set_drvdata(pdev, dev);

	v4l2_info(&dev->v4l2_dev, "Media entity: %s\n", vfd->entity.name);
	v4l2_info(&dev->v4l2_dev,
		  "Device registered as %s, on ipu%d\n",
		  video_device_node_name(vfd), ipu_get_num(dev->ipu));
	return 0;

cleanup_ctx:
	if (!IS_ERR_OR_NULL(dev->alloc_ctx))
		vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
unreg_vdev:
	video_unregister_device(&dev->vfd);
	return ret;
}

static int mx6cam_graph_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct mx6cam_dev *camdev =
		container_of(notifier, struct mx6cam_dev, notifier);
	struct mx6cam_graph_entity *entity;
/*	struct v4l2_dv_timings timings;
	struct v4l2_subdev_format sd_fmt = {
		.pad = 1,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.format.code = MEDIA_BUS_FMT_YUYV8_1X16,
	};*/
	int ret;

	dev_dbg(camdev->dev, "notify complete, all subdevs registered\n");

	/* Create links for every entity. */
	list_for_each_entry(entity, &camdev->entities, list) {
		ret = mx6cam_graph_build_one(camdev, entity);
		if (ret < 0) {
			dev_dbg(camdev->dev, "graph build interrupted, error=%d\n", ret);
			return ret;
		}
	}

	ret = v4l2_device_register_subdev_nodes(&camdev->v4l2_dev);
	if (ret < 0)
		dev_err(camdev->dev, "failed to register subdev nodes\n");
	else {
		camdev->ep = &camdev->eplist[0];
		list_for_each_entry(entity, &camdev->entities, list) {
			if (strcmp(camdev->ep->sd->name, "adv7611 1-004c") == 0) {
				dev_dbg(camdev->dev, "Entity %s found, now set format\n", camdev->ep->sd->name);
			}
		}
/*		ret = v4l2_subdev_call(camdev->ep->sd,
			video, s_dv_timings, &timings);
		if (ret < 0)
			dev_dbg(camdev->dev, "Could not set dv_timings on %s\n", camdev->ep->sd->name);
		ret = v4l2_subdev_call(camdev->ep->sd, pad, set_fmt, NULL, &sd_fmt);
		if (ret < 0)
			dev_dbg(camdev->dev, "Could not set format on %s\n", camdev->ep->sd->name);

		camdev->format.pixelformat = V4L2_PIX_FMT_YUYV;
		camdev->dv_timings_cap = timings;
		update_format_from_timings(camdev, &timings);*/
	}
	ret = mx6cam_probe_complete(camdev);
	return ret;
}

static int mx6cam_graph_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct mx6cam_dev *camdev =
		container_of(notifier, struct mx6cam_dev, notifier);
	struct mx6cam_graph_entity *entity;

	/* Locate the entity corresponding to the bound subdev and store the
	 * subdev pointer
	 */

	list_for_each_entry(entity, &camdev->entities, list) {
		if (entity->node != subdev->dev->of_node)
			continue;

		if (entity->subdev) {
			dev_err(camdev->dev, "duplicate subdev for node %s\n",
				entity->node->full_name);
			return -EINVAL;
		}

		dev_dbg(camdev->dev, "subdev %s bound\n", subdev->name);
		entity->entity = &subdev->entity;
		entity->subdev = subdev;
		camdev->eplist[camdev->num_eps].sd = subdev;
		camdev->num_eps++;
		return 0;
	}

	dev_err(camdev->dev, "no entity for subdev %s\n", subdev->name);
	return -EINVAL;
}

static int mx6cam_graph_parse_one(struct mx6cam_dev *camdev,
				struct device_node *node)
{
	struct mx6cam_graph_entity *entity;
	struct device_node *remote;
	struct device_node *ep = NULL;
//	struct v4l2_of_endpoint v4l2_ep;
	struct device_node *next;
	int ret = 0;

	dev_dbg(camdev->dev, "parsing node %s\n", node->full_name);

	while (1) {
		next = of_graph_get_next_endpoint(node, ep);
		if (next == NULL)
			break;

		of_node_put(ep);
		ep = next;

		dev_dbg(camdev->dev, "handling endpoint %s\n", ep->full_name);
		if (camdev->csi_id == -1) {
			ret = of_property_read_u32(ep, "reg", &camdev->csi_id);
			if (!ret)
				dev_dbg(camdev->dev, "add CSI %d\n", camdev->csi_id);
			else
				dev_dbg(camdev->dev, "Could not find reg property : %d\n", ret);
		}

//		v4l2_of_parse_endpoint(ep, &v4l2_ep);
		remote = of_graph_get_remote_port_parent(ep);
		if (remote == NULL) {
			ret = -EINVAL;
			dev_dbg(camdev->dev, "remote invalid\n");
			break;
		}

		entity = devm_kzalloc(camdev->dev, sizeof(*entity), GFP_KERNEL);
		if (entity == NULL) {
			of_node_put(remote);
			ret = -ENOMEM;
			break;
		}

		/* Add root node to entities but not as an async dev */
		if (remote == camdev->dev->of_node) {
			entity->node = remote;
			entity->subdev = NULL;
			entity->entity = &camdev->vfd.entity;
			list_add_tail(&entity->list, &camdev->entities);
			continue;
		}

		/* Skip entities that we have already processed. */
		if (mx6cam_graph_find_entity(camdev, remote)) {
			of_node_put(remote);
			dev_dbg(camdev->dev, "entity already processed %s\n",
				remote->full_name);
			continue;
		}

		entity->node = remote;
		entity->asd.match_type = V4L2_ASYNC_MATCH_OF;
		entity->asd.match.of.node = remote;
		list_add_tail(&entity->list, &camdev->entities);
		dev_dbg(camdev->dev, "entity %s added\n",
			remote->full_name);
		camdev->num_subdevs++;
	}

	of_node_put(ep);
	return ret;
}

static int mx6cam_graph_parse(struct mx6cam_dev *camdev)
{
	struct mx6cam_graph_entity *entity;
	int ret;

	/*
	* Walk the links to parse the full graph. Start by parsing the
	* composite node and then parse entities in turn. The list_for_each
	* loop will handle entities added at the end of the list while walking
	* the links.
	*/
	ret = mx6cam_graph_parse_one(camdev, camdev->dev->of_node);
	if (ret < 0)
		return 0;

	list_for_each_entry(entity, &camdev->entities, list) {
		ret = mx6cam_graph_parse_one(camdev, entity->node);
		if (ret < 0)
			break;
	}

	return ret;
}

static void mx6cam_graph_cleanup(struct mx6cam_dev *camdev)
{
	struct mx6cam_graph_entity *entityp;
	struct mx6cam_graph_entity *entity;

	v4l2_async_notifier_unregister(&camdev->notifier);

	list_for_each_entry_safe(entity, entityp, &camdev->entities, list) {
		of_node_put(entity->node);
		list_del(&entity->list);
	}
}

static int mx6cam_graph_init(struct mx6cam_dev *camdev)
{
	struct mx6cam_graph_entity *entity;
	struct v4l2_async_subdev **subdevs = NULL;
	unsigned int num_subdevs;
	unsigned int i;
	int ret;

	/* Parse the graph to extract a list of subdevice DT nodes. */
	ret = mx6cam_graph_parse(camdev);
	if (ret < 0) {
		dev_err(camdev->dev, "graph parsing failed\n");
		goto done;
	}

	if (!camdev->num_subdevs) {
		dev_err(camdev->dev, "no subdev found in graph\n");
		goto done;
	}

	/* Register the subdevices notifier */
	num_subdevs = camdev->num_subdevs;
	subdevs = devm_kzalloc(camdev->dev, sizeof(*subdevs) * num_subdevs,
				GFP_KERNEL);
	if (subdevs == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	i = 0;
	list_for_each_entry(entity, &camdev->entities, list)
		subdevs[i++] = &entity->asd;

	camdev->notifier.subdevs = subdevs;
	camdev->notifier.num_subdevs = num_subdevs;
	camdev->notifier.bound = mx6cam_graph_notify_bound;
	camdev->notifier.complete = mx6cam_graph_notify_complete;

	ret = v4l2_async_notifier_register(&camdev->v4l2_dev,
					&camdev->notifier);
	if (ret < 0) {
		dev_err(camdev->dev, "notifier registration failed\n");
		goto done;
	}

ret = 0;

done:
	if (ret < 0)
		mx6cam_graph_cleanup(camdev);
	return ret;
}

static int mx6cam_probe(struct platform_device *pdev)
{
	struct mx6_camera_pdata *pdata = pdev->dev.platform_data;
	struct device_node *node = pdev->dev.of_node;
	struct mx6cam_dev *dev;
	struct video_device *vfd;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	mutex_init(&dev->mutex);
	spin_lock_init(&dev->irqlock);

	dev->media_dev.dev = dev->dev;
	strlcpy(dev->media_dev.model, "i.MX6 Video Capture Device",
		sizeof(dev->media_dev.model));
	dev->media_dev.hw_revision = 0;

	ret = media_device_register(&dev->media_dev);
	if (ret < 0) {
		dev_err(dev->dev, "media device registration failed (%d)\n",
			ret);
		return ret;
	}
	dev->v4l2_dev.mdev = &dev->media_dev;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret < 0) {
		dev_err(dev->dev, "V4L2 device registration failed (%d)\n",
			ret);
		media_device_unregister(&dev->media_dev);
		return ret;
	}

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if (!pdata) {
		v4l2_err(&dev->v4l2_dev, "no platform data!\n");
		ret = -EINVAL;
		goto unreg_dev;
	}
	dev->pdata = pdata;

	/* get our IPU */
	dev->ipu = mx6cam_find_ipu(dev, node);
	if (IS_ERR_OR_NULL(dev->ipu)) {
		v4l2_err(&dev->v4l2_dev, "could not get ipu\n");
		ret = -ENODEV;
		goto unreg_dev;
	}

	vfd = &dev->vfd;
	INIT_LIST_HEAD(&dev->entities);
	dev->ep = NULL;
	dev->csi_id = -1;
	dev->pads[0].flags = MEDIA_PAD_FL_SINK;
	dev->pads[1].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vfd->entity, 2, dev->pads, 0);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "media entity failed with error %d\n", ret);
		goto unreg_dev;
	}

	/* find and register mipi csi2 receiver subdev */
	ret = mx6cam_add_csi2_receiver(dev);
	if (ret)
		goto free_ctrls;
#if 0
	/* parse and register all sensor endpoints */
	ret = mx6cam_parse_endpoints(dev, node);
	if (ret)
		goto unreg_subdevs;
#endif
	ret = mx6cam_graph_init(dev);
	if (ret)
		goto unreg_subdevs;
	if (!dev->ep)
		ret = -EPROBE_DEFER;

	dev->encoder_sd = mx6cam_encoder_init(dev);
	if (IS_ERR(dev->encoder_sd)) {
		ret = PTR_ERR(dev->encoder_sd);
		goto unreg_subdevs;
	}
#if 0
	dev->preview_sd = mx6cam_preview_init(dev);
	if (IS_ERR(dev->preview_sd)) {
		ret = PTR_ERR(dev->preview_sd);
		goto unreg_subdevs;
	}

	dev->vdic_sd = mx6cam_vdic_init(dev);
	if (IS_ERR(dev->vdic_sd)) {
		ret = PTR_ERR(dev->vdic_sd);
		goto unreg_subdevs;
	}
#endif
	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->encoder_sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to register encoder subdev\n");
		goto unreg_subdevs;
	}
	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->encoder_sd->name);
#if 0
	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->preview_sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to register preview subdev\n");
		goto unreg_subdevs;
	}
	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->preview_sd->name);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->vdic_sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to register vdic subdev\n");
		goto unreg_subdevs;
	}
	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->vdic_sd->name);
#endif
	return 0;

unreg_subdevs:
	mx6cam_unregister_subdevs(dev);
free_ctrls:
	v4l2_ctrl_handler_free(&dev->ctrl_hdlr);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}

static int mx6cam_remove(struct platform_device *pdev)
{
	struct mx6cam_dev *dev =
		(struct mx6cam_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " DEVICE_NAME "\n");
	v4l2_ctrl_handler_free(&dev->ctrl_hdlr);
	video_unregister_device(&dev->vfd);
	if (!IS_ERR_OR_NULL(dev->alloc_ctx))
		vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	mx6cam_unregister_subdevs(dev);
	mx6cam_graph_cleanup(dev);
	v4l2_device_unregister(&dev->v4l2_dev);

	return 0;
}

static struct of_device_id mx6cam_dt_ids[] = {
	{ .compatible = "fsl,imx6-v4l2-capture" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mx6cam_dt_ids);

static struct platform_driver mx6cam_pdrv = {
	.probe		= mx6cam_probe,
	.remove		= mx6cam_remove,
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= mx6cam_dt_ids,
	},
};

module_platform_driver(mx6cam_pdrv);

MODULE_DESCRIPTION("i.MX6 v4l2 capture driver");
MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_LICENSE("GPL");
