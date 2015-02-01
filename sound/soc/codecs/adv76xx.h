/*
 * Copyright (C) 2014 Vodalys-Labs
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

#ifndef ADV76XX_H_
#define ADV76XX_H_

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/adv7604.h>

#define DAI_ADV7611_NAME	"adv7611-dai"
#define DAI_ADV7604_NAME	"adv7604-dai"
#define PLATFORM_DRIVER_NAME	"adv76xx-asoc-codec"

enum adv7604_type {
	ADV7604,
	ADV7611,
};

struct adv7604_chip_info {
	enum adv7604_type type;

	bool has_afe;
	unsigned int max_port;
	unsigned int num_dv_ports;

	unsigned int edid_enable_reg;
	unsigned int edid_status_reg;
	unsigned int lcf_reg;

	unsigned int cable_det_mask;
	unsigned int tdms_lock_mask;
	unsigned int fmt_change_digital_mask;

	const struct adv7604_format_info *formats;
	unsigned int nformats;

	void (*set_termination)(struct v4l2_subdev *sd, bool enable);
	void (*setup_irqs)(struct v4l2_subdev *sd);
	unsigned int (*read_hdmi_pixelclock)(struct v4l2_subdev *sd);
	unsigned int (*read_cable_det)(struct v4l2_subdev *sd);
	int (*init_core)(struct v4l2_subdev *sd);

	/* 0 = AFE, 1 = HDMI */
	const struct adv7604_reg_seq *recommended_settings[2];
	unsigned int num_recommended_settings[2];

	unsigned long page_mask;
};

/* Struct with dai and codec information */
struct adv76xx_snd_data
{
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct adv7604_state *state;
};


#endif /* ADV7611_CODEC_H_ */
