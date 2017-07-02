/*
 * ddbridge-irq.c: Digital Devices bridge irq handlers
 *
 * Copyright (C) 2010-2015 Digital Devices GmbH
 *                         Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/swab.h>
#include <linux/vmalloc.h>

#include "ddbridge.h"
#include "ddbridge-regs.h"

/******************************************************************************/

#define IRQ_HANDLE(_nr) \
	do { if ((s & (1UL << ((_nr) & 0x1f))) && dev->handler[0][_nr]) \
		dev->handler[0][_nr](dev->handler_data[0][_nr]); } \
	while (0)

#define IRQ_HANDLE_BYTE(_n) \
	if (s & (0x000000ff << ((_n) & 0x1f))) { \
		IRQ_HANDLE(0 + _n); \
		IRQ_HANDLE(1 + _n); \
		IRQ_HANDLE(2 + _n); \
		IRQ_HANDLE(3 + _n); \
		IRQ_HANDLE(4 + _n); \
		IRQ_HANDLE(5 + _n); \
		IRQ_HANDLE(6 + _n); \
		IRQ_HANDLE(7 + _n); \
	}

static void irq_handle_msg(struct ddb *dev, u32 s)
{
	dev->i2c_irq++;
	IRQ_HANDLE(0);
	IRQ_HANDLE(1);
	IRQ_HANDLE(2);
	IRQ_HANDLE(3);
}

static void irq_handle_io(struct ddb *dev, u32 s)
{
	dev->ts_irq++;
	if ((s & 0x000000f0)) {
		IRQ_HANDLE(4);
		IRQ_HANDLE(5);
		IRQ_HANDLE(6);
		IRQ_HANDLE(7);
	}
	if ((s & 0x0000ff00)) {
		IRQ_HANDLE(8);
		IRQ_HANDLE(9);
		IRQ_HANDLE(10);
		IRQ_HANDLE(11);
		IRQ_HANDLE(12);
		IRQ_HANDLE(13);
		IRQ_HANDLE(14);
		IRQ_HANDLE(15);
	}
	if ((s & 0x00ff0000)) {
		IRQ_HANDLE(16);
		IRQ_HANDLE(17);
		IRQ_HANDLE(18);
		IRQ_HANDLE(19);
		IRQ_HANDLE(20);
		IRQ_HANDLE(21);
		IRQ_HANDLE(22);
		IRQ_HANDLE(23);
	}
	if ((s & 0xff000000)) {
		IRQ_HANDLE(24);
		IRQ_HANDLE(25);
		IRQ_HANDLE(26);
		IRQ_HANDLE(27);
		IRQ_HANDLE(28);
		IRQ_HANDLE(29);
		IRQ_HANDLE(30);
		IRQ_HANDLE(31);
	}
}

irqreturn_t irq_handler0(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		if (!(s & 0xfff00))
			return IRQ_NONE;
		ddbwritel(dev, s & 0xfff00, INTERRUPT_ACK);
		irq_handle_io(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));

	return IRQ_HANDLED;
}

irqreturn_t irq_handler1(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);

	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		if (!(s & 0x0000f))
			return IRQ_NONE;
		ddbwritel(dev, s & 0x0000f, INTERRUPT_ACK);
		irq_handle_msg(dev, s);
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));

	return IRQ_HANDLED;
}

irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(dev, INTERRUPT_STATUS);
	int ret = IRQ_HANDLED;

	if (!s)
		return IRQ_NONE;
	do {
		if (s & 0x80000000)
			return IRQ_NONE;
		ddbwritel(dev, s, INTERRUPT_ACK);

		if (s & 0x0000000f)
			irq_handle_msg(dev, s);
		if (s & 0x0fffff00) {
			irq_handle_io(dev, s);
		}
	} while ((s = ddbreadl(dev, INTERRUPT_STATUS)));

	return ret;
}
