// SPDX-License-Identifier: GPL-2.0
/*
 * ddbridge-ioctl.c: Digital Devices bridge IOCTL handler
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/vmalloc.h>

#include "ddbridge.h"
#include "ddbridge-i2c.h"
#include "ddbridge-regs.h"
#include "ddbridge-io.h"
#include "ddbridge-ioctl.h"

/******************************************************************************/

static int reg_wait(struct ddb *dev, u32 reg, u32 bit)
{
	u32 count = 0;

	while (safe_ddbreadl(dev, reg) & bit) {
		ndelay(10);
		if (++count == 100)
			return -1;
	}
	return 0;
}

/******************************************************************************/

int ddb_do_flashio(struct ddb *dev, u32 lnr, u8 *wbuf, u32 wlen,
		   u8 *rbuf, u32 rlen)
{
	u32 data, shift;
	u32 tag = DDB_LINK_TAG(lnr);
	struct ddb_link *link = &dev->link[lnr];

	mutex_lock(&link->flash_mutex);
	if (wlen > 4)
		ddbwritel(dev, 1, tag | SPI_CONTROL);
	while (wlen > 4) {
		/* FIXME: check for big-endian */
		data = swab32(*(u32 *)wbuf);
		wbuf += 4;
		wlen -= 4;
		ddbwritel(dev, data, tag | SPI_DATA);
		if (reg_wait(dev, tag | SPI_CONTROL, 4))
			goto fail;
	}
	if (rlen)
		ddbwritel(dev, 0x0001 | ((wlen << (8 + 3)) & 0x1f00),
			  tag | SPI_CONTROL);
	else
		ddbwritel(dev, 0x0003 | ((wlen << (8 + 3)) & 0x1f00),
			  tag | SPI_CONTROL);

	data = 0;
	shift = ((4 - wlen) * 8);
	while (wlen) {
		data <<= 8;
		data |= *wbuf;
		wlen--;
		wbuf++;
	}
	if (shift)
		data <<= shift;
	ddbwritel(dev, data, tag | SPI_DATA);
	if (reg_wait(dev, tag | SPI_CONTROL, 4))
		goto fail;

	if (!rlen) {
		ddbwritel(dev, 0, tag | SPI_CONTROL);
		goto exit;
	}
	if (rlen > 4)
		ddbwritel(dev, 1, tag | SPI_CONTROL);

	while (rlen > 4) {
		ddbwritel(dev, 0xffffffff, tag | SPI_DATA);
		if (reg_wait(dev, tag | SPI_CONTROL, 4))
			goto fail;
		data = ddbreadl(dev, tag | SPI_DATA);
		*(u32 *)rbuf = swab32(data);
		rbuf += 4;
		rlen -= 4;
	}
	ddbwritel(dev, 0x0003 | ((rlen << (8 + 3)) & 0x1F00),
		  tag | SPI_CONTROL);
	ddbwritel(dev, 0xffffffff, tag | SPI_DATA);
	if (reg_wait(dev, tag | SPI_CONTROL, 4))
		goto fail;

	data = ddbreadl(dev, tag | SPI_DATA);
	ddbwritel(dev, 0, tag | SPI_CONTROL);

	if (rlen < 4)
		data <<= ((4 - rlen) * 8);

	while (rlen > 0) {
		*rbuf = ((data >> 24) & 0xff);
		data <<= 8;
		rbuf++;
		rlen--;
	}
exit:
	mutex_unlock(&link->flash_mutex);
	return 0;
fail:
	mutex_unlock(&link->flash_mutex);
	return -1;
}

long ddb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ddb *dev = file->private_data;
	__user void *parg = (__user void *)arg;
	int res;

	switch (cmd) {
	case DDB_IOCTL_FLASHIO:
	{
		struct ddb_flashio fio;
		u8 *rbuf, *wbuf;

		if (copy_from_user(&fio, parg, sizeof(fio)))
			return -EFAULT;
		if (fio.write_len > 1028 || fio.read_len > 1028)
			return -EINVAL;
		if (fio.write_len + fio.read_len > 1028)
			return -EINVAL;
		if (fio.link > 3)
			return -EINVAL;

		wbuf = dev->iobuf;
		rbuf = wbuf + fio.write_len;

		if (copy_from_user(wbuf, fio.write_buf, fio.write_len))
			return -EFAULT;
		res = ddb_do_flashio(dev, fio.link, wbuf, fio.write_len,
				     rbuf, fio.read_len);
		if (res)
			return res;
		if (copy_to_user(fio.read_buf, rbuf, fio.read_len))
			return -EFAULT;
		break;
	}
	case DDB_IOCTL_ID:
	{
		struct ddb_id ddbid;

		ddbid.vendor = dev->link[0].ids.vendor;
		ddbid.device = dev->link[0].ids.device;
		ddbid.subvendor = dev->link[0].ids.subvendor;
		ddbid.subdevice = dev->link[0].ids.subdevice;
		ddbid.hw = ddbreadl(dev, 0);
		ddbid.regmap = ddbreadl(dev, 4);
		if (copy_to_user(parg, &ddbid, sizeof(ddbid)))
			return -EFAULT;
		break;
	}
	default:
		return -ENOTTY;
	}
	return 0;
}
