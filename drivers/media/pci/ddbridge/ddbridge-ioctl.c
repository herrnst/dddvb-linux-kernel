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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
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

static int mdio_write(struct ddb *dev, u8 adr, u8 reg, u16 val)
{
	ddbwritel(dev, adr, MDIO_ADR);
	ddbwritel(dev, reg, MDIO_REG);
	ddbwritel(dev, val, MDIO_VAL);
	ddbwritel(dev, 0x03, MDIO_CTRL);
	while (ddbreadl(dev, MDIO_CTRL) & 0x02)
		ndelay(500);
	return 0;
}

static u16 mdio_read(struct ddb *dev, u8 adr, u8 reg)
{
	ddbwritel(dev, adr, MDIO_ADR);
	ddbwritel(dev, reg, MDIO_REG);
	ddbwritel(dev, 0x07, MDIO_CTRL);
	while (ddbreadl(dev, MDIO_CTRL) & 0x02)
		ndelay(500);
	return ddbreadl(dev, MDIO_VAL);
}

/******************************************************************************/

int ddb_flashio(struct ddb *dev, u32 lnr, u8 *wbuf, u32 wlen, u8 *rbuf,
		u32 rlen)
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
	case IOCTL_DDB_FLASHIO:
	{
		struct ddb_flash_io fio;
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
		res = ddb_flashio(dev, fio.link, wbuf, fio.write_len,
				  rbuf, fio.read_len);
		if (res)
			return res;
		if (copy_to_user(fio.read_buf, rbuf, fio.read_len))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_GPIO_OUT:
	{
		struct ddb_gpio gpio;

		if (copy_from_user(&gpio, parg, sizeof(gpio)))
			return -EFAULT;
		ddbwritel(dev, gpio.mask, GPIO_DIRECTION);
		ddbwritel(dev, gpio.data, GPIO_OUTPUT);
		break;
	}
	case IOCTL_DDB_ID:
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
	case IOCTL_DDB_READ_REG:
	{
		struct ddb_reg reg;

		if (copy_from_user(&reg, parg, sizeof(reg)))
			return -EFAULT;
		if ((reg.reg & 0xfffffff) >= dev->regs_len)
			return -EINVAL;
		reg.val = ddbreadl(dev, reg.reg);
		if (copy_to_user(parg, &reg, sizeof(reg)))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_REG:
	{
		struct ddb_reg reg;

		if (copy_from_user(&reg, parg, sizeof(reg)))
			return -EFAULT;
		if ((reg.reg & 0xfffffff) >= dev->regs_len)
			return -EINVAL;
		ddbwritel(dev, reg.val, reg.reg);
		break;
	}
	case IOCTL_DDB_READ_MDIO:
	{
		struct ddb_mdio mdio;

		if (!dev->link[0].info->mdio_num)
			return -EIO;
		if (copy_from_user(&mdio, parg, sizeof(mdio)))
			return -EFAULT;
		mdio.val = mdio_read(dev, mdio.adr, mdio.reg);
		if (copy_to_user(parg, &mdio, sizeof(mdio)))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_MDIO:
	{
		struct ddb_mdio mdio;

		if (!dev->link[0].info->mdio_num)
			return -EIO;
		if (copy_from_user(&mdio, parg, sizeof(mdio)))
			return -EFAULT;
		mdio_write(dev, mdio.adr, mdio.reg, mdio.val);
		break;
	}
	case IOCTL_DDB_READ_MEM:
	{
		struct ddb_mem mem;
		u8 *buf = dev->iobuf;

		if (copy_from_user(&mem, parg, sizeof(mem)))
			return -EFAULT;
		if ((((mem.len + mem.off) & 0xfffffff) > dev->regs_len) ||
		    mem.len > 1024)
			return -EINVAL;
		ddbcpyfrom(dev, buf, mem.off, mem.len);
		if (copy_to_user(mem.buf, buf, mem.len))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_MEM:
	{
		struct ddb_mem mem;
		u8 *buf = dev->iobuf;

		if (copy_from_user(&mem, parg, sizeof(mem)))
			return -EFAULT;
		if ((((mem.len + mem.off) & 0xfffffff) > dev->regs_len) ||
		    mem.len > 1024)
			return -EINVAL;
		if (copy_from_user(buf, mem.buf, mem.len))
			return -EFAULT;
		ddbcpyto(dev, mem.off, buf, mem.len);
		break;
	}
	case IOCTL_DDB_READ_I2C:
	{
		struct ddb_i2c_msg i2c;
		struct i2c_adapter *adap;
		u8 *mbuf, *hbuf = dev->iobuf;

		if (copy_from_user(&i2c, parg, sizeof(i2c)))
			return -EFAULT;
		if (i2c.bus > dev->link[0].info->regmap->i2c->num)
			return -EINVAL;
		if (i2c.mlen + i2c.hlen > 512)
			return -EINVAL;

		adap = &dev->i2c[i2c.bus].adap;
		mbuf = hbuf + i2c.hlen;

		if (copy_from_user(hbuf, i2c.hdr, i2c.hlen))
			return -EFAULT;
		if (i2c_io(adap, i2c.adr, hbuf, i2c.hlen, mbuf, i2c.mlen) < 0)
			return -EIO;
		if (copy_to_user(i2c.msg, mbuf, i2c.mlen))
			return -EFAULT;
		break;
	}
	case IOCTL_DDB_WRITE_I2C:
	{
		struct ddb_i2c_msg i2c;
		struct i2c_adapter *adap;
		u8 *buf = dev->iobuf;

		if (copy_from_user(&i2c, parg, sizeof(i2c)))
			return -EFAULT;
		if (i2c.bus > dev->link[0].info->regmap->i2c->num)
			return -EINVAL;
		if (i2c.mlen + i2c.hlen > 250)
			return -EINVAL;

		adap = &dev->i2c[i2c.bus].adap;
		if (copy_from_user(buf, i2c.hdr, i2c.hlen))
			return -EFAULT;
		if (copy_from_user(buf + i2c.hlen, i2c.msg, i2c.mlen))
			return -EFAULT;
		if (i2c_write(adap, i2c.adr, buf, i2c.hlen + i2c.mlen) < 0)
			return -EIO;
		break;
	}
	default:
		return -ENOTTY;
	}
	return 0;
}
