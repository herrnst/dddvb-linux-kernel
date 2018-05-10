/* SPDX-License-Identifier: GPL-2.0 */
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_DDBRIDGE_IOCTL_H__
#define __LINUX_DDBRIDGE_IOCTL_H__

#include <linux/compiler.h>
#include <linux/types.h>

/******************************************************************************/

#define DDB_IOCTL_MAGIC		0xDD
#define DDB_IOCTL_SEQIDX	0xC0

/* DDB_IOCTL_FLASHIO */
struct ddb_flashio {
	/* write_*: userspace -> flash */
	__user __u8 *write_buf;
	__u32        write_len;
	/* read_*: flash -> userspace */
	__user __u8 *read_buf;
	__u32        read_len;
	/* card/addon link */
	__u32        link;
};

/* DDB_IOCTL_GPIO_{IN,OUT} */
struct ddb_gpio {
	__u32 mask;
	__u32 data;
};

/* DDB_IOCTL_ID */
struct ddb_id {
	/* card/PCI device data, FPGA/regmap info */
	__u16 vendor;
	__u16 device;
	__u16 subvendor;
	__u16 subdevice;
	__u32 hw;
	__u32 regmap;
};

/* DDB_IOCTL_{READ,WRITE}_REG */
struct ddb_reg {
	__u32 reg;
	__u32 val;
};

/* DDB_IOCTL_{READ,WRITE}_MEM */
struct ddb_mem {
	/* read/write card mem, off = offset from base address */
	__u32        off;
	__user __u8 *buf;
	__u32        len;
};

/* DDB_IOCTL_{READ,WRITE}_MDIO */
struct ddb_mdio {
	/* read/write from/to MDIO address/register/value */
	__u8  adr;
	__u8  reg;
	__u16 val;
};

/* DDB_IOCTL_{READ,WRITE}_I2C */
struct ddb_i2c_msg {
	/* card's I2C bus number */
	__u8   bus;
	/* I2C address */
	__u8   adr;
	/* I2C IO header */
	__u8  *hdr;
	__u32  hlen;
	/* I2C message */
	__u8  *msg;
	__u32  mlen;
};

/* IOCTLs */
#define DDB_IOCTL_FLASHIO \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x00), struct ddb_flashio)
#define DDB_IOCTL_GPIO_IN \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x01), struct ddb_gpio)
#define DDB_IOCTL_GPIO_OUT \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x02), struct ddb_gpio)
#define DDB_IOCTL_ID \
	_IOR(DDB_IOCTL_MAGIC,  (DDB_IOCTL_SEQIDX + 0x03), struct ddb_id)
#define DDB_IOCTL_READ_REG \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x04), struct ddb_reg)
#define DDB_IOCTL_WRITE_REG \
	_IOW(DDB_IOCTL_MAGIC,  (DDB_IOCTL_SEQIDX + 0x05), struct ddb_reg)
#define DDB_IOCTL_READ_MEM \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x06), struct ddb_mem)
#define DDB_IOCTL_WRITE_MEM \
	_IOR(DDB_IOCTL_MAGIC,  (DDB_IOCTL_SEQIDX + 0x07), struct ddb_mem)
#define DDB_IOCTL_READ_MDIO \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x08), struct ddb_mdio)
#define DDB_IOCTL_WRITE_MDIO \
	_IOR(DDB_IOCTL_MAGIC,  (DDB_IOCTL_SEQIDX + 0x09), struct ddb_mdio)
#define DDB_IOCTL_READ_I2C \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x0A), struct ddb_i2c_msg)
#define DDB_IOCTL_WRITE_I2C \
	_IOR(DDB_IOCTL_MAGIC,  (DDB_IOCTL_SEQIDX + 0x0B), struct ddb_i2c_msg)

/******************************************************************************/

#endif /* __LINUX_DDBRIDGE_IOCTL_H__ */
