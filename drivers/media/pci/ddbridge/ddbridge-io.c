/*
 * ddbridge-i2c.c: Digital Devices bridge i2c driver
 *
 * Copyright (C) 2010-2017 Digital Devices GmbH
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

u32 ddblreadl(struct ddb_link *link, u32 adr)
{
	return readl((char *) (link->dev->regs + (adr)));
}

void ddblwritel(struct ddb_link *link, u32 val, u32 adr)
{
	writel(val, (char *) (link->dev->regs + (adr)));
}

u32 ddbreadl(struct ddb *dev, u32 adr)
{
	return readl((char *) (dev->regs + (adr)));
}

void ddbwritel(struct ddb *dev, u32 val, u32 adr)
{
	writel(val, (char *) (dev->regs + (adr)));
}

void ddbcpyto(struct ddb *dev, u32 adr, void *src, long count)
{
	return memcpy_toio((char *) (dev->regs + adr), src, count);
}

void ddbcpyfrom(struct ddb *dev, void *dst, u32 adr, long count)
{
	return memcpy_fromio(dst, (char *) (dev->regs + adr), count);
}

u32 safe_ddbreadl(struct ddb *dev, u32 adr)
{
	u32 val = ddbreadl(dev, adr);

	/* (ddb)readl returns (uint)-1 (all bits set) on failure, catch that */
	if (val == ~0) {
		dev_err(&dev->pdev->dev, "ddbreadl failure, adr=%08x\n", adr);
		return 0;
	}

	return val;
}
