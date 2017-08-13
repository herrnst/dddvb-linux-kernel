/*
 * ddbridge-ioctl.h: Digital Devices bridge IOCTL handler
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

#ifndef __DDBRIDGE_IOCTL_H__
#define __DDBRIDGE_IOCTL_H__

#include <linux/ddbridge-ioctl.h>

#include "ddbridge.h"

/******************************************************************************/

int ddb_flashio(struct ddb *dev, u32 lnr, u8 *wbuf, u32 wlen, u8 *rbuf,
		u32 rlen);
long ddb_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#endif /* __DDBRIDGE_IOCTL_H__ */
