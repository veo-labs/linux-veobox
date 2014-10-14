/*
 * addi_apci_3120.c
 * Copyright (C) 2004,2005  ADDI-DATA GmbH for the source code of this module.
 *
 *	ADDI-DATA GmbH
 *	Dieselstrasse 3
 *	D-77833 Ottersweier
 *	Tel: +19(0)7223/9493-0
 *	Fax: +49(0)7223/9493-92
 *	http://www.addi-data.com
 *	info@addi-data.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#include "../comedidev.h"
#include "comedi_fc.h"
#include "amcc_s5933.h"

/*
 * PCI BAR 0 register map (devpriv->amcc)
 * see amcc_s5933.h for register and bit defines
 */
#define APCI3120_FIFO_ADVANCE_ON_BYTE_2		(1 << 29)

/*
 * PCI BAR 1 register map (dev->iobase)
 */
#define APCI3120_AI_FIFO_REG			0x00
#define APCI3120_CTRL_REG			0x00
#define APCI3120_CTRL_EXT_TRIG			(1 << 15)
#define APCI3120_CTRL_GATE(x)			(1 << (12 + (x)))
#define APCI3120_CTRL_PR(x)			(((x) & 0xf) << 8)
#define APCI3120_CTRL_PA(x)			(((x) & 0xf) << 0)
#define APCI3120_AI_SOFTTRIG_REG		0x02
#define APCI3120_STATUS_REG			0x02
#define APCI3120_STATUS_EOC_INT			(1 << 15)
#define APCI3120_STATUS_AMCC_INT		(1 << 14)
#define APCI3120_STATUS_EOS_INT			(1 << 13)
#define APCI3120_STATUS_TIMER2_INT		(1 << 12)
#define APCI3120_STATUS_INT_MASK		(0xf << 12)
#define APCI3120_STATUS_TO_DI_BITS(x)		(((x) >> 8) & 0xf)
#define APCI3120_STATUS_TO_VERSION(x)		(((x) >> 4) & 0xf)
#define APCI3120_STATUS_FIFO_FULL		(1 << 2)
#define APCI3120_STATUS_FIFO_EMPTY		(1 << 1)
#define APCI3120_STATUS_DA_READY		(1 << 0)
#define APCI3120_TIMER_REG			0x04
#define APCI3120_CHANLIST_REG			0x06
#define APCI3120_CHANLIST_INDEX(x)		(((x) & 0xf) << 8)
#define APCI3120_CHANLIST_UNIPOLAR		(1 << 7)
#define APCI3120_CHANLIST_GAIN(x)		(((x) & 0x3) << 4)
#define APCI3120_CHANLIST_MUX(x)		(((x) & 0xf) << 0)
#define APCI3120_AO_REG(x)			(0x08 + (((x) / 4) * 2))
#define APCI3120_AO_MUX(x)			(((x) & 0x3) << 14)
#define APCI3120_AO_DATA(x)			((x) << 0)
#define APCI3120_TIMER_MODE_REG			0x0c
#define APCI3120_TIMER_MODE(_t, _m)		((_m) << ((_t) * 2))
#define APCI3120_TIMER_MODE0			0  /* I8254_MODE0 */
#define APCI3120_TIMER_MODE2			1  /* I8254_MODE2 */
#define APCI3120_TIMER_MODE4			2  /* I8254_MODE4 */
#define APCI3120_TIMER_MODE5			3  /* I8254_MODE5 */
#define APCI3120_TIMER_MODE_MASK(_t)		(3 << ((_t) * 2))
#define APCI3120_CTR0_REG			0x0d
#define APCI3120_CTR0_DO_BITS(x)		((x) << 4)
#define APCI3120_CTR0_TIMER_SEL(x)		((x) << 0)
#define APCI3120_MODE_REG			0x0e
#define APCI3120_MODE_TIMER2_CLK_OSC		(0 << 6)
#define APCI3120_MODE_TIMER2_CLK_OUT1		(1 << 6)
#define APCI3120_MODE_TIMER2_CLK_EOC		(2 << 6)
#define APCI3120_MODE_TIMER2_CLK_EOS		(3 << 6)
#define APCI3120_MODE_TIMER2_CLK_MASK		(3 << 6)
#define APCI3120_MODE_TIMER2_AS_TIMER		(0 << 4)
#define APCI3120_MODE_TIMER2_AS_COUNTER		(1 << 4)
#define APCI3120_MODE_TIMER2_AS_WDOG		(2 << 4)
#define APCI3120_MODE_TIMER2_AS_MASK		(3 << 4)  /* sets AS_TIMER */
#define APCI3120_MODE_SCAN_ENA			(1 << 3)
#define APCI3120_MODE_TIMER2_IRQ_ENA		(1 << 2)
#define APCI3120_MODE_EOS_IRQ_ENA		(1 << 1)
#define APCI3120_MODE_EOC_IRQ_ENA		(1 << 0)

/*
 * PCI BAR 2 register map (devpriv->addon)
 */
#define APCI3120_ADDON_ADDR_REG			0x00
#define APCI3120_ADDON_DATA_REG			0x02
#define APCI3120_ADDON_CTRL_REG			0x04
#define APCI3120_ADDON_CTRL_AMWEN_ENA		(1 << 1)
#define APCI3120_ADDON_CTRL_A2P_FIFO_ENA	(1 << 0)

/*
 * Board revisions
 */
#define APCI3120_REVA				0xa
#define APCI3120_REVB				0xb
#define APCI3120_REVA_OSC_BASE			70	/* 70ns = 14.29MHz */
#define APCI3120_REVB_OSC_BASE			50	/* 50ns = 20MHz */

static const struct comedi_lrange apci3120_ai_range = {
	8, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2),
		BIP_RANGE(1),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2),
		UNI_RANGE(1)
	}
};

enum apci3120_boardid {
	BOARD_APCI3120,
	BOARD_APCI3001,
};

struct apci3120_board {
	const char *name;
	int i_AiChannelList;
	int i_NbrAoChannel;
	int i_AiMaxdata;
	int i_AoMaxdata;
};

static const struct apci3120_board apci3120_boardtypes[] = {
	[BOARD_APCI3120] = {
		.name			= "apci3120",
		.i_AiChannelList	= 16,
		.i_NbrAoChannel		= 8,
		.i_AiMaxdata		= 0xffff,
		.i_AoMaxdata		= 0x3fff,
	},
	[BOARD_APCI3001] = {
		.name			= "apci3001",
		.i_AiChannelList	= 16,
		.i_AiMaxdata		= 0xfff,
	},
};

static int apci3120_auto_attach(struct comedi_device *dev,
				unsigned long context)
{
	struct pci_dev *pcidev = comedi_to_pci_dev(dev);
	const struct apci3120_board *this_board = NULL;
	struct addi_private *devpriv;
	struct comedi_subdevice *s;
	unsigned int status;
	int ret;

	if (context < ARRAY_SIZE(apci3120_boardtypes))
		this_board = &apci3120_boardtypes[context];
	if (!this_board)
		return -ENODEV;
	dev->board_ptr = this_board;
	dev->board_name = this_board->name;

	devpriv = comedi_alloc_devpriv(dev, sizeof(*devpriv));
	if (!devpriv)
		return -ENOMEM;

	ret = comedi_pci_enable(dev);
	if (ret)
		return ret;
	pci_set_master(pcidev);

	dev->iobase = pci_resource_start(pcidev, 1);
	devpriv->amcc = pci_resource_start(pcidev, 0);
	devpriv->addon = pci_resource_start(pcidev, 2);

	apci3120_reset(dev);

	if (pcidev->irq > 0) {
		ret = request_irq(pcidev->irq, apci3120_interrupt, IRQF_SHARED,
				  dev->board_name, dev);
		if (ret == 0) {
			dev->irq = pcidev->irq;

			apci3120_dma_alloc(dev);
		}
	}

	status = inw(dev->iobase + APCI3120_STATUS_REG);
	if (APCI3120_STATUS_TO_VERSION(status) == APCI3120_REVB ||
	    context == BOARD_APCI3001)
		devpriv->osc_base = APCI3120_REVB_OSC_BASE;
	else
		devpriv->osc_base = APCI3120_REVA_OSC_BASE;

	ret = comedi_alloc_subdevices(dev, 5);
	if (ret)
		return ret;

	/* Analog Input subdevice */
	s = &dev->subdevices[0];
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags =
		SDF_READABLE | SDF_COMMON | SDF_GROUND
		| SDF_DIFF;
	s->n_chan = 16;
	s->maxdata = this_board->i_AiMaxdata;
	s->len_chanlist = this_board->i_AiChannelList;
	s->range_table = &range_apci3120_ai;

	s->insn_config = apci3120_ai_insn_config;
	s->insn_read = apci3120_ai_insn_read;
	s->do_cmdtest = apci3120_ai_cmdtest;
	s->do_cmd = apci3120_ai_cmd;
	s->cancel = apci3120_cancel;

	/*  Allocate and Initialise AO Subdevice Structures */
	s = &dev->subdevices[1];
	if (this_board->has_ao) {
		s->type		= COMEDI_SUBD_AO;
		s->subdev_flags	= SDF_WRITABLE | SDF_GROUND | SDF_COMMON;
		s->n_chan	= 8;
		s->maxdata	= 0x3fff;
		s->range_table	= &range_bipolar10;
		s->insn_write	= apci3120_ao_insn_write;

		ret = comedi_alloc_subdev_readback(s);
		if (ret)
			return ret;
	} else {
		s->type		= COMEDI_SUBD_UNUSED;
	}

	/* Digital Input subdevice */
	s = &dev->subdevices[2];
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = apci3120_di_insn_bits;

	/*  Allocate and Initialise DO Subdevice Structures */
	s = &dev->subdevices[3];
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags =
		SDF_READABLE | SDF_WRITEABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = apci3120_do_insn_bits;

	/*  Allocate and Initialise Timer Subdevice Structures */
	s = &dev->subdevices[4];
	s->type		= COMEDI_SUBD_TIMER;
	s->subdev_flags	= SDF_READABLE;
	s->n_chan	= 1;
	s->maxdata	= 0x00ffffff;
	s->insn_config	= apci3120_timer_insn_config;
	s->insn_read	= apci3120_timer_insn_read;

	return 0;
}

static void apci3120_detach(struct comedi_device *dev)
{
	comedi_pci_detach(dev);
	apci3120_dma_free(dev);
}

static struct comedi_driver apci3120_driver = {
	.driver_name	= "addi_apci_3120",
	.module		= THIS_MODULE,
	.auto_attach	= apci3120_auto_attach,
	.detach		= apci3120_detach,
};

static int apci3120_pci_probe(struct pci_dev *dev,
			      const struct pci_device_id *id)
{
	return comedi_pci_auto_config(dev, &apci3120_driver, id->driver_data);
}

static const struct pci_device_id apci3120_pci_table[] = {
	{ PCI_VDEVICE(AMCC, 0x818d), BOARD_APCI3120 },
	{ PCI_VDEVICE(AMCC, 0x828d), BOARD_APCI3001 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, apci3120_pci_table);

static struct pci_driver apci3120_pci_driver = {
	.name		= "addi_apci_3120",
	.id_table	= apci3120_pci_table,
	.probe		= apci3120_pci_probe,
	.remove		= comedi_pci_auto_unconfig,
};
module_comedi_pci_driver(apci3120_driver, apci3120_pci_driver);

MODULE_AUTHOR("Comedi http://www.comedi.org");
MODULE_DESCRIPTION("ADDI-DATA APCI-3120, Analog input board");
MODULE_LICENSE("GPL");
