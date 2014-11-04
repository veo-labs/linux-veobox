/*
 * Video Capture driver for Freescale i.MX6 SOC
 *
 * Copyright (c) 2012-2014 Mentor Graphics Inc.
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _MX6_CAMIF_H
#define _MX6_CAMIF_H

#define dprintk(dev, fmt, arg...)					\
	v4l2_dbg(1, 1, &dev->v4l2_dev, "%s: " fmt, __func__, ## arg)

/*
 * There can be a maximum of 5 endpoints (and 5 sensors attached to those
 * endpoints): 1 parallel endpoints, and 4 MIPI-CSI2 endpoints for each
 * virtual channel.
 */
#define MX6CAM_MAX_ENDPOINTS 5

/*
 * How long before no EOF interrupts cause a stream/preview
 * restart, or a buffer dequeue timeout, in msec. The dequeue
 * timeout should be longer than the EOF timeout.
 */
#define MX6CAM_EOF_TIMEOUT       1000
#define MX6CAM_DQ_TIMEOUT        5000

/*
 * How long to delay a restart on ADV718x status changes or NFB4EOF,
 * in msec.
 */
#define MX6CAM_RESTART_DELAY      200

/*
 * Internal subdev notifications
 */
#define MX6CAM_NFB4EOF_NOTIFY      _IO('6', 0)
#define MX6CAM_EOF_TIMEOUT_NOTIFY  _IO('6', 1)

struct mx6cam_buffer {
	struct vb2_buffer vb; /* v4l buffer must be first */
	struct list_head  list;
};

static inline struct mx6cam_buffer *to_mx6cam_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct mx6cam_buffer, vb);
}

struct mx6cam_pixfmt {
	char	*name;
	u32	fourcc;
	u32     codes[4];
	int     depth;   /* total bpp */
	int     y_depth; /* depth of first Y plane for planar formats */
};

struct mx6cam_dma_buf {
	void          *virt;
	dma_addr_t     phys;
	unsigned long  len;
};

/*
 * A sensor's inputs parsed from v4l2_of_endpoint nodes in devicetree
 */
#define MX6CAM_MAX_INPUTS 16

struct mx6cam_sensor_input {
	/* input values passed to s_routing */
	u32 value[MX6CAM_MAX_INPUTS];
	/* input capabilities (V4L2_IN_CAP_*) */
	u32 caps[MX6CAM_MAX_INPUTS];
	/* input names */
	char name[MX6CAM_MAX_INPUTS][32];

	/* number of inputs */
	int num;
	/* first and last input indexes from mx6cam perspective */
	int first;
	int last;
};

/*
 * Everything to describe a V4L2 endpoint. Endpoints are handled by
 * one of the two CSI's, and connect to exactly one remote sensor.
 */
struct mx6cam_endpoint {
	struct v4l2_of_endpoint ep;      /* the parsed DT endpoint info */
	struct v4l2_subdev     *sd;      /* the remote sensor when attached
					    to this endpoint */
	struct mx6cam_sensor_input sensor_input;
	int power_count;                 /* power use counter */
	int stream_count;                /* stream use counter */
};

struct mx6cam_ctx;

struct mx6cam_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd;
	struct device           *dev;
	struct mx6_camera_pdata *pdata;

	struct mutex		mutex;
	spinlock_t		irqlock;

	/* buffer queue used in videobuf2 */
	struct vb2_queue        buffer_queue;
	void *alloc_ctx;

	/* v4l2 controls */
	struct v4l2_ctrl_handler ctrl_hdlr;
	int                      rotation; /* degrees */
	bool                     hflip;
	bool                     vflip;
	enum ipu_motion_sel      motion;

	/* derived from rotation, hflip, vflip controls */
	enum ipu_rotate_mode     rot_mode;

	/* the format from sensor and from userland */
	struct v4l2_format        user_fmt;
	struct v4l2_pix_format	format;
	struct mx6cam_pixfmt      *user_pixfmt;
	struct v4l2_mbus_framefmt sensor_fmt;
	struct mx6cam_pixfmt      *sensor_pixfmt;
	struct v4l2_mbus_config   mbus_cfg;

	/*
	 * win (from s_fmt_vid_cap_overlay) holds global alpha, chromakey,
	 * and interlace info for the preview overlay.
	 */
	struct v4l2_window      win;

	/*
	 * info about the overlay framebuffer for preview (base address,
	 * width/height, pix format).
	 */
	struct v4l2_framebuffer fbuf;
	struct mx6cam_pixfmt    *fbuf_pixfmt;

	/*
	 * the crop rectangle (from s_crop) specifies the crop dimensions
	 * and position over the raw capture frame boundaries.
	 */
	struct v4l2_rect        crop_bounds;
	struct v4l2_rect        crop_defrect;
	struct v4l2_rect        crop;

	/* misc status */
	int                     current_input; /* the current input */
	v4l2_std_id             current_std;   /* current video standard */
	atomic_t                status_change; /* sensor status change */
	bool                    signal_locked; /* sensor signal lock */
	bool                    encoder_on;    /* encode is on */
	bool                    preview_on;    /* preview is on */
	bool                    using_ic;      /* IC is being used for encode */
	bool                    using_vdic;    /* VDIC is used for encode */

	/* encoder, preview, vdic, and mipi csi2 subdevices */
	struct v4l2_subdev     *encoder_sd;
	struct v4l2_subdev     *preview_sd;
	struct v4l2_subdev     *vdic_sd;
	struct v4l2_subdev     *csi2_sd;

	/* sensor endpoints */
	struct mx6cam_endpoint  eplist[MX6CAM_MAX_ENDPOINTS];
	struct mx6cam_endpoint  *ep; /* the current active endpoint */
	int                     num_eps;

	/*
	 * the current open context that is doing IO (there can only
	 * be one allowed IO context at a time).
	 */
	struct mx6cam_ctx       *io_ctx;

	struct ipu_soc          *ipu;
};

struct mx6cam_ctx {
	struct v4l2_fh          fh;
	struct mx6cam_dev       *dev;

	/* streaming buffer queue */
	struct list_head        ready_q;

	/* stream/preview stop and restart handling */
	struct work_struct      restart_work;
	struct work_struct      stop_work;
	struct timer_list       restart_timer;

	/* is this ctx allowed to do IO */
	bool                    io_allowed;
};

struct v4l2_subdev *mx6cam_encoder_init(struct mx6cam_dev *dev);
struct v4l2_subdev *mx6cam_preview_init(struct mx6cam_dev *dev);
struct v4l2_subdev *mx6cam_vdic_init(struct mx6cam_dev *dev);

#endif /* _MX6_CAMIF_H */
