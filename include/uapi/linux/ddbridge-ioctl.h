/*
 * ddbridge-ioctl.h: Digital Devices bridge IOCTL API
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

#ifndef __LINUX_DDBRIDGE_IOCTL_H__
#define __LINUX_DDBRIDGE_IOCTL_H__

#include <linux/compiler.h>
#include <linux/types.h>

/******************************************************************************/

#define DDB_MAGIC 'd'

struct ddb_flash_io {
	__user __u8 *write_buf;
	__u32 write_len;
	__user __u8 *read_buf;
	__u32 read_len;
	__u32 link;
};

struct ddb_gpio {
	__u32 mask;
	__u32 data;
};

struct ddb_id {
	__u16 vendor;
	__u16 device;
	__u16 subvendor;
	__u16 subdevice;
	__u32 hw;
	__u32 regmap;
};

struct ddb_reg {
	__u32 reg;
	__u32 val;
};

struct ddb_mem {
	__u32  off;
	__user __u8  *buf;
	__u32  len;
};

struct ddb_mdio {
	__u8   adr;
	__u8   reg;
	__u16  val;
};

struct ddb_i2c_msg {
	__u8   bus;
	__u8   adr;
	__u8  *hdr;
	__u32  hlen;
	__u8  *msg;
	__u32  mlen;
};

#define IOCTL_DDB_FLASHIO	_IOWR(DDB_MAGIC, 0x00, struct ddb_flash_io)
#define IOCTL_DDB_GPIO_IN	_IOWR(DDB_MAGIC, 0x01, struct ddb_gpio)
#define IOCTL_DDB_GPIO_OUT	_IOWR(DDB_MAGIC, 0x02, struct ddb_gpio)
#define IOCTL_DDB_ID		_IOR(DDB_MAGIC, 0x03, struct ddb_id)
#define IOCTL_DDB_READ_REG	_IOWR(DDB_MAGIC, 0x04, struct ddb_reg)
#define IOCTL_DDB_WRITE_REG	_IOW(DDB_MAGIC, 0x05, struct ddb_reg)
#define IOCTL_DDB_READ_MEM	_IOWR(DDB_MAGIC, 0x06, struct ddb_mem)
#define IOCTL_DDB_WRITE_MEM	_IOR(DDB_MAGIC, 0x07, struct ddb_mem)
#define IOCTL_DDB_READ_MDIO	_IOWR(DDB_MAGIC, 0x08, struct ddb_mdio)
#define IOCTL_DDB_WRITE_MDIO	_IOR(DDB_MAGIC, 0x09, struct ddb_mdio)
#define IOCTL_DDB_READ_I2C	_IOWR(DDB_MAGIC, 0x0a, struct ddb_i2c_msg)
#define IOCTL_DDB_WRITE_I2C	_IOR(DDB_MAGIC, 0x0b, struct ddb_i2c_msg)

/******************************************************************************/

#endif /* __LINUX_DDBRIDGE_IOCTL_H__ */
