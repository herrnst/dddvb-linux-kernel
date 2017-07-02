/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ddbridge-io.h: Digital Devices bridge I/O inline functions
 *
 * Copyright (C) 2010-2017 Digital Devices GmbH
 *                         Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 */

#ifndef __DDBRIDGE_IO_H__
#define __DDBRIDGE_IO_H__

#include <linux/io.h>

#include "ddbridge.h"

/******************************************************************************/

#define ddbmemset(_dev, _adr, _val, _count) \
	memset_io((char *)(_dev->regs + (_adr)), (_val), (_count))

/******************************************************************************/

u32 ddblreadl(struct ddb_link *link, u32 adr);
void ddblwritel(struct ddb_link *link, u32 val, u32 adr);
u32 ddbreadl(struct ddb *dev, u32 adr);
void ddbwritel(struct ddb *dev, u32 val, u32 adr);
void ddbcpyto(struct ddb *dev, u32 adr, void *src, long count);
void ddbcpyfrom(struct ddb *dev, void *dst, u32 adr, long count);

/******************************************************************************/

static inline void ddbwriteb(struct ddb *dev, u32 val, u32 adr)
{
	writeb(val, dev->regs + adr);
}

static inline u32 ddbreadb(struct ddb *dev, u32 adr)
{
	return readb(dev->regs + adr);
}

static inline void ddbwritel0(struct ddb_link *link, u32 val, u32 adr)
{
	writel(val, link->dev->regs + adr);
}

static inline u32 ddbreadl0(struct ddb_link *link, u32 adr)
{
	return readl(link->dev->regs + adr);
}

static inline void gtlw(struct ddb_link *link)
{
	while (1 & ddbreadl0(link, link->regs + 0x10))
		;
}

static inline u32 safe_ddbreadl(struct ddb *dev, u32 adr)
{
	u32 val = ddbreadl(dev, adr);

	/* (ddb)readl returns (uint)-1 (all bits set) on failure, catch that */
	if (val == ~0) {
		dev_err(&dev->pdev->dev, "ddbreadl failure, adr=%08x\n", adr);
		return 0;
	}

	return val;
}

#endif /* __DDBRIDGE_IO_H__ */
