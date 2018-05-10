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
#define DDB_IOCTL_SEQIDX	0xE0

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

/* IOCTLs */
#define DDB_IOCTL_FLASHIO \
	_IOWR(DDB_IOCTL_MAGIC, (DDB_IOCTL_SEQIDX + 0x00), struct ddb_flashio)
#define DDB_IOCTL_ID \
	_IOR(DDB_IOCTL_MAGIC,  (DDB_IOCTL_SEQIDX + 0x03), struct ddb_id)

/******************************************************************************/

#endif /* __LINUX_DDBRIDGE_IOCTL_H__ */
