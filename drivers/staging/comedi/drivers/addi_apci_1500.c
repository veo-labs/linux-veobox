/*
 * addi_apci_1500.c
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
#include "z8536.h"

/*
 * PCI Bar 0 Register map (devpriv->amcc)
 * see amcc_s5933.h for register and bit defines
 */

/*
 * PCI Bar 1 Register map (dev->iobase)
 * see z8536.h for Z8536 internal registers and bit defines
 */
#define APCI1500_Z8536_PORTC_REG	0x00
#define APCI1500_Z8536_PORTB_REG	0x01
#define APCI1500_Z8536_PORTA_REG	0x02
#define APCI1500_Z8536_CTRL_REG		0x03

/*
 * PCI Bar 2 Register map (devpriv->addon)
 */
#define APCI1500_CLK_SEL_REG		0x00
#define APCI1500_DI_REG			0x00
#define APCI1500_DO_REG			0x02

struct apci1500_private {
	unsigned long amcc;
	unsigned long addon;

	unsigned int clk_src;

	/* Digital trigger configuration [0]=AND [1]=OR */
	unsigned int pm[2];	/* Pattern Mask */
	unsigned int pt[2];	/* Pattern Transition */
	unsigned int pp[2];	/* Pattern Polarity */
};

#include "addi-data/hwdrv_apci1500.c"

static int apci1500_auto_attach(struct comedi_device *dev,
				unsigned long context)
{
	struct pci_dev *pcidev = comedi_to_pci_dev(dev);
	struct addi_private *devpriv;
	struct comedi_subdevice *s;
	int ret;

	devpriv = comedi_alloc_devpriv(dev, sizeof(*devpriv));
	if (!devpriv)
		return -ENOMEM;

	ret = comedi_pci_enable(dev);
	if (ret)
		return ret;

	dev->iobase = pci_resource_start(pcidev, 1);
	devpriv->iobase = dev->iobase;
	devpriv->i_IobaseAmcc = pci_resource_start(pcidev, 0);
	devpriv->i_IobaseAddon = pci_resource_start(pcidev, 2);
	devpriv->i_IobaseReserved = pci_resource_start(pcidev, 3);

	if (pcidev->irq > 0) {
		ret = request_irq(pcidev->irq, apci1500_interrupt, IRQF_SHARED,
				  dev->board_name, dev);
		if (ret == 0)
			dev->irq = pcidev->irq;
	}

	ret = comedi_alloc_subdevices(dev, 3);
	if (ret)
		return ret;

	/*  Allocate and Initialise DI Subdevice Structures */
	s = &dev->subdevices[0];
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 16;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_config = apci1500_di_config;
	s->insn_read = apci1500_di_read;
	s->insn_write = apci1500_di_write;
	s->insn_bits = apci1500_di_insn_bits;

	/*  Allocate and Initialise DO Subdevice Structures */
	s = &dev->subdevices[1];
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags =
		SDF_READABLE | SDF_WRITEABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 16;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_config = apci1500_do_config;
	s->insn_write = apci1500_do_write;
	s->insn_bits = apci1500_do_bits;

	/*  Allocate and Initialise Timer Subdevice Structures */
	s = &dev->subdevices[2];
	s->type = COMEDI_SUBD_TIMER;
	s->subdev_flags = SDF_WRITEABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 1;
	s->maxdata = 0;
	s->len_chanlist = 1;
	s->range_table = &range_digital;
	s->insn_write = apci1500_timer_write;
	s->insn_read = apci1500_timer_read;
	s->insn_config = apci1500_timer_config;
	s->insn_bits = apci1500_timer_bits;

	apci1500_reset(dev);

	return 0;
}

static void apci1500_detach(struct comedi_device *dev)
{
	struct apci1500_private *devpriv = dev->private;

	if (devpriv->amcc)
		outl(0x0, devpriv->amcc + AMCC_OP_REG_INTCSR);
	comedi_pci_detach(dev);
}

static struct comedi_driver apci1500_driver = {
	.driver_name	= "addi_apci_1500",
	.module		= THIS_MODULE,
	.auto_attach	= apci1500_auto_attach,
	.detach		= apci1500_detach,
};

static int apci1500_pci_probe(struct pci_dev *dev,
			      const struct pci_device_id *id)
{
	return comedi_pci_auto_config(dev, &apci1500_driver, id->driver_data);
}

static const struct pci_device_id apci1500_pci_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_AMCC, 0x80fc) },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, apci1500_pci_table);

static struct pci_driver apci1500_pci_driver = {
	.name		= "addi_apci_1500",
	.id_table	= apci1500_pci_table,
	.probe		= apci1500_pci_probe,
	.remove		= comedi_pci_auto_unconfig,
};
module_comedi_pci_driver(apci1500_driver, apci1500_pci_driver);

MODULE_AUTHOR("Comedi http://www.comedi.org");
MODULE_DESCRIPTION("ADDI-DATA APCI-1500, 16 channel DI / 16 channel DO boards");
MODULE_LICENSE("GPL");
