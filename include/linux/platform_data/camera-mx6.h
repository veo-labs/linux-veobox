/*
 * Platform data for i.MX6 Video Capture driver
 *
 * Copyright (c) 2014 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MX6_CAMERA_H_
#define _MX6_CAMERA_H_

/**
 * struct mx6_camera_pdata - i.MX6 camera platform data
 * @set_video_mux: Set video iomux
 */
struct mx6_camera_pdata {
	void (*set_video_mux)(int ipu, int csi, bool csi2, u32 vc);
};

#endif
