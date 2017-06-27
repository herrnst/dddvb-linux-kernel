/*
 * ddbridge.c: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2011 Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * To obtain the license, point your browser to
 * http://www.gnu.org/copyleft/gpl.html
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

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

int i2c_io(struct i2c_adapter *adapter, u8 adr,
		  u8 *wbuf, u32 wlen, u8 *rbuf, u32 rlen)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = wbuf, .len   = wlen },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = rbuf,  .len   = rlen } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_write(struct i2c_adapter *adap, u8 adr, u8 *data, int len)
{
	struct i2c_msg msg = {.addr = adr, .flags = 0,
			      .buf = data, .len = len};

	return (i2c_transfer(adap, &msg, 1) == 1) ? 0 : -1;
}

int i2c_read(struct i2c_adapter *adapter, u8 adr, u8 *val)
{
	struct i2c_msg msgs[1] = {{.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 1) == 1) ? 0 : -1;
}

int i2c_read_regs(struct i2c_adapter *adapter,
			 u8 adr, u8 reg, u8 *val, u8 len)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = &reg, .len   = 1 },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = len } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

int i2c_read_reg(struct i2c_adapter *adapter, u8 adr, u8 reg, u8 *val)
{
	return i2c_read_regs(adapter, adr, reg, val, 1);
}

int i2c_read_reg16(struct i2c_adapter *adapter, u8 adr,
			  u16 reg, u8 *val)
{
	u8 msg[2] = {reg>>8, reg&0xff};
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = msg, .len   = 2},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = 1} };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

int i2c_write_reg(struct i2c_adapter *adap, u8 adr,
			 u8 reg, u8 val)
{
	u8 msg[2] = {reg, val};

	return i2c_write(adap, adr, msg, 2);
}

static int ddb_i2c_cmd(struct ddb_i2c *i2c, u32 adr, u32 cmd)
{
	struct ddb *dev = i2c->dev;
	long stat;
	u32 val;

	i2c->done = 0;
	ddbwritel((adr << 9) | cmd, i2c->regs + I2C_COMMAND);
	stat = wait_event_timeout(i2c->wq, i2c->done == 1, HZ);
	if (stat == 0) {
		dev_err(&dev->pdev->dev, "I2C timeout\n");
		{ /* MSI debugging*/
			u32 istat = ddbreadl(INTERRUPT_STATUS);
			dev_err(&dev->pdev->dev, "IRS %08x\n", istat);
			ddbwritel(istat, INTERRUPT_ACK);
		}
		return -EIO;
	}
	val = ddbreadl(i2c->regs+I2C_COMMAND);
	if (val & 0x70000)
		return -EIO;
	return 0;
}

static int ddb_i2c_master_xfer(struct i2c_adapter *adapter,
			       struct i2c_msg msg[], int num)
{
	struct ddb_i2c *i2c = (struct ddb_i2c *)i2c_get_adapdata(adapter);
	struct ddb *dev = i2c->dev;
	u8 addr = 0;

	if (num)
		addr = msg[0].addr;

	if (num == 2 && msg[1].flags & I2C_M_RD &&
	    !(msg[0].flags & I2C_M_RD)) {
		memcpy_toio(dev->regs + I2C_TASKMEM_BASE + i2c->wbuf,
			    msg[0].buf, msg[0].len);
		ddbwritel(msg[0].len|(msg[1].len << 16),
			  i2c->regs+I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 1)) {
			memcpy_fromio(msg[1].buf,
				      dev->regs + I2C_TASKMEM_BASE + i2c->rbuf,
				      msg[1].len);
			return num;
		}
	}

	if (num == 1 && !(msg[0].flags & I2C_M_RD)) {
		ddbcpyto(I2C_TASKMEM_BASE + i2c->wbuf, msg[0].buf, msg[0].len);
		ddbwritel(msg[0].len, i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 2))
			return num;
	}
	if (num == 1 && (msg[0].flags & I2C_M_RD)) {
		ddbwritel(msg[0].len << 16, i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 3)) {
			ddbcpyfrom(msg[0].buf,
				   I2C_TASKMEM_BASE + i2c->rbuf, msg[0].len);
			return num;
		}
	}
	return -EIO;
}


static u32 ddb_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm ddb_i2c_algo = {
	.master_xfer   = ddb_i2c_master_xfer,
	.functionality = ddb_i2c_functionality,
};

void ddb_i2c_release(struct ddb *dev)
{
	int i;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i = 0; i < dev->info->port_num; i++) {
		i2c = &dev->i2c[i];
		adap = &i2c->adap;
		i2c_del_adapter(adap);
	}
}

int ddb_i2c_init(struct ddb *dev)
{
	int i, j, stat = 0;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i = 0; i < dev->info->port_num; i++) {
		i2c = &dev->i2c[i];
		i2c->dev = dev;
		i2c->nr = i;
		i2c->wbuf = i * (I2C_TASKMEM_SIZE / 4);
		i2c->rbuf = i2c->wbuf + (I2C_TASKMEM_SIZE / 8);
		i2c->regs = 0x80 + i * 0x20;
		ddbwritel(I2C_SPEED_100, i2c->regs + I2C_TIMING);
		ddbwritel((i2c->rbuf << 16) | i2c->wbuf,
			  i2c->regs + I2C_TASKADDRESS);
		init_waitqueue_head(&i2c->wq);

		adap = &i2c->adap;
		i2c_set_adapdata(adap, i2c);
#ifdef I2C_ADAP_CLASS_TV_DIGITAL
		adap->class = I2C_ADAP_CLASS_TV_DIGITAL|I2C_CLASS_TV_ANALOG;
#else
#ifdef I2C_CLASS_TV_ANALOG
		adap->class = I2C_CLASS_TV_ANALOG;
#endif
#endif
		strcpy(adap->name, "ddbridge");
		adap->algo = &ddb_i2c_algo;
		adap->algo_data = (void *)i2c;
		adap->dev.parent = &dev->pdev->dev;
		stat = i2c_add_adapter(adap);
		if (stat)
			break;
	}
	if (stat)
		for (j = 0; j < i; j++) {
			i2c = &dev->i2c[j];
			adap = &i2c->adap;
			i2c_del_adapter(adap);
		}
	return stat;
}
