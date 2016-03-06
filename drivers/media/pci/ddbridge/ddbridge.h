/*
 * ddbridge.h: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2015 Digital Devices GmbH
 *                         Ralph Metzler <rmetzler@digitaldevices.de>
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

#ifndef _DDBRIDGE_H_
#define _DDBRIDGE_H_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/swab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/completion.h>

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <linux/dvb/ca.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/io.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_ringbuffer.h"
#include "dvb_ca_en50221.h"
#include "dvb_net.h"

#include "tda18271c2dd.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"
#include "drxk.h"
#include "stv0367.h"
#include "stv0367_priv.h"
#include "cxd2841er.h"
#include "tda18212.h"
#include "cxd2099.h"
#include "stv0910.h"
#include "stv6111.h"
#include "lnbh25.h"
#include "mxl5xx.h"

#define DDB_MAX_I2C    32
#define DDB_MAX_PORT   32
#define DDB_MAX_INPUT  64
#define DDB_MAX_OUTPUT 32
#define DDB_MAX_LINK    4
#define DDB_LINK_SHIFT 28

#define DDB_LINK_TAG(_x) (_x << DDB_LINK_SHIFT)

struct ddb_regset {
	u32 base;
	u32 num;
	u32 size;
};

struct ddb_regmap {
	u32 irq_base_i2c;
	u32 irq_base_idma;
	u32 irq_base_odma;
	u32 irq_base_gtl;

	struct ddb_regset *i2c;
	struct ddb_regset *i2c_buf;
	struct ddb_regset *idma;
	struct ddb_regset *idma_buf;
	struct ddb_regset *odma;
	struct ddb_regset *odma_buf;

	struct ddb_regset *input;
	struct ddb_regset *output;

	struct ddb_regset *channel;
	struct ddb_regset *gtl;
};

struct ddb_ids {
	u16 vendor;
	u16 device;
	u16 subvendor;
	u16 subdevice;

	u32 hwid;
	u32 regmapid;
	u32 devid;
	u32 mac;
};

struct ddb_info {
	int   type;
#define DDB_NONE         0
#define DDB_OCTOPUS      1
#define DDB_OCTOPUS_CI   2
#define DDB_OCTOPUS_MAX  5
#define DDB_OCTOPUS_MAX_CT  6
	char *name;
	u32   i2c_mask;
	u8    port_num;
	u8    led_num;
	u8    fan_num;
	u8    temp_num;
	u8    temp_bus;
	u32   board_control;
	u32   board_control_2;
	u8    mdio_num;
	u8    con_clock; /* use a continuous clock */
	u8    ts_quirks;
#define TS_QUIRK_SERIAL   1
#define TS_QUIRK_REVERSED 2
#define TS_QUIRK_ALT_OSC  8
	u32   tempmon_irq;
	struct ddb_regmap *regmap;
};

/* DMA_SIZE MUST be smaller than 256k and
 * MUST be divisible by 188 and 128 !!!
 */

#define DMA_MAX_BUFS 32      /* hardware table limit */

#define INPUT_DMA_BUFS 8
#define INPUT_DMA_SIZE (128*47*21)
#define INPUT_DMA_IRQ_DIV 1

#define OUTPUT_DMA_BUFS 8
#define OUTPUT_DMA_SIZE (128*47*21)
#define OUTPUT_DMA_IRQ_DIV 1

struct ddb;
struct ddb_port;

struct ddb_dma {
	void                  *io;
	u32                    regs;
	u32                    bufregs;

	dma_addr_t             pbuf[DMA_MAX_BUFS];
	u8                    *vbuf[DMA_MAX_BUFS];
	u32                    num;
	u32                    size;
	u32                    div;
	u32                    bufval;

#ifdef DDB_USE_WORK
	struct work_struct     work;
#else
	struct tasklet_struct  tasklet;
#endif
	spinlock_t             lock;
	wait_queue_head_t      wq;
	int                    running;
	u32                    stat;
	u32                    ctrl;
	u32                    cbuf;
	u32                    coff;
};

struct ddb_dvb {
	struct dvb_adapter    *adap;
	int                    adap_registered;
	struct dvb_device     *dev;
	struct i2c_client     *i2c_client[1];
	struct dvb_frontend   *fe;
	struct dvb_frontend   *fe2;
	struct dmxdev          dmxdev;
	struct dvb_demux       demux;
	struct dvb_net         dvbnet;
	struct dmx_frontend    hw_frontend;
	struct dmx_frontend    mem_frontend;
	int                    users;
	u32                    attached;
	u8                     input;

	enum fe_sec_tone_mode  tone;
	enum fe_sec_voltage    voltage;

	int (*i2c_gate_ctrl)(struct dvb_frontend *, int);
	int (*set_voltage)(struct dvb_frontend *fe,
		enum fe_sec_voltage voltage);
	int (*set_input)(struct dvb_frontend *fe, int input);
	int (*diseqc_send_master_cmd)(struct dvb_frontend *fe,
		struct dvb_diseqc_master_cmd *cmd);
};

struct ddb_ci {
	struct dvb_ca_en50221  en;
	struct ddb_port       *port;
	u32                    nr;
	struct mutex           lock;
};

struct ddb_io {
	struct ddb_port       *port;
	u32                    nr;
	u32                    regs;
	struct ddb_dma        *dma;
	struct ddb_io         *redo;
	struct ddb_io         *redi;
};

#define ddb_output ddb_io
#define ddb_input ddb_io

struct ddb_i2c {
	struct ddb            *dev;
	u32                    nr;
	u32                    regs;
	u32                    link;
	struct i2c_adapter     adap;
	u32                    rbuf;
	u32                    wbuf;
	u32                    bsize;
	struct completion      completion;
};

struct ddb_port {
	struct ddb            *dev;
	u32                    nr;
	u32                    pnr;
	u32                    regs;
	u32                    lnr;
	struct ddb_i2c        *i2c;
	struct mutex           i2c_gate_lock;
	u32                    class;
#define DDB_PORT_NONE           0
#define DDB_PORT_CI             1
#define DDB_PORT_TUNER          2
#define DDB_PORT_LOOP           3
	char                   *name;
	char                   *type_name;
	u32                     type;
#define DDB_TUNER_NONE           0
#define DDB_TUNER_DVBS_ST        1
#define DDB_TUNER_DVBS_ST_AA     2
#define DDB_TUNER_DVBCT_TR       3
#define DDB_TUNER_DVBCT_ST       4
#define DDB_CI_INTERNAL          5
#define DDB_CI_EXTERNAL_SONY     6
#define DDB_TUNER_DVBCT2_SONY_P  7
#define DDB_TUNER_DVBC2T2_SONY_P 8
#define DDB_TUNER_ISDBT_SONY_P   9
#define DDB_TUNER_DVBS_STV0910_P 10
#define DDB_TUNER_MXL5XX         11
#define DDB_CI_EXTERNAL_XO2      12
#define DDB_CI_EXTERNAL_XO2_B    13
#define DDB_TUNER_DVBS_STV0910_PR 14
#define DDB_TUNER_DVBC2T2I_SONY_P 15

#define DDB_TUNER_XO2            32
#define DDB_TUNER_DVBS_STV0910   (DDB_TUNER_XO2 + 0)
#define DDB_TUNER_DVBCT2_SONY    (DDB_TUNER_XO2 + 1)
#define DDB_TUNER_ISDBT_SONY     (DDB_TUNER_XO2 + 2)
#define DDB_TUNER_DVBC2T2_SONY   (DDB_TUNER_XO2 + 3)
#define DDB_TUNER_ATSC_ST        (DDB_TUNER_XO2 + 4)
#define DDB_TUNER_DVBC2T2I_SONY  (DDB_TUNER_XO2 + 5)

	struct ddb_input      *input[2];
	struct ddb_output     *output;
	struct dvb_ca_en50221 *en;
	struct ddb_dvb         dvb[2];
	u32                    gap;
	u32                    obr;
	u8                     creg;
};

#define CM_STARTUP_DELAY 2
#define CM_AVERAGE  20
#define CM_GAIN     10

#define HW_LSB_SHIFT    12
#define HW_LSB_MASK     0x1000

#define CM_IDLE    0
#define CM_STARTUP 1
#define CM_ADJUST  2

#define TS_CAPTURE_LEN  (4096)

struct ddb_lnb {
	struct mutex           lock;
	u32                    tone;
	enum fe_sec_voltage    oldvoltage[4];
	u32                    voltage[4];
	u32                    voltages;
	u32                    fmode;
};

struct ddb_link {
	struct ddb            *dev;
	struct ddb_info       *info;
	u32                    nr;
	u32                    regs;
	spinlock_t             lock;
	struct mutex           flash_mutex;
	struct ddb_lnb         lnb;
	struct tasklet_struct  tasklet;
	struct ddb_ids         ids;

	spinlock_t             temp_lock;
	int                    OverTemperatureError;
	u8                     temp_tab[11];
};

struct ddb {
	struct pci_dev        *pdev;
	struct platform_device *pfdev;
	struct device         *dev;

	int                    msi;
	struct workqueue_struct *wq;
	u32                    has_dma;

	struct ddb_link        link[DDB_MAX_LINK];
	unsigned char         *regs;
	u32                    regs_len;
	u32                    port_num;
	struct ddb_port        port[DDB_MAX_PORT];
	u32                    i2c_num;
	struct ddb_i2c         i2c[DDB_MAX_I2C];
	struct ddb_input       input[DDB_MAX_INPUT];
	struct ddb_output      output[DDB_MAX_OUTPUT];
	struct dvb_adapter     adap[DDB_MAX_INPUT];
	struct ddb_dma         idma[DDB_MAX_INPUT];
	struct ddb_dma         odma[DDB_MAX_OUTPUT];

	void                   (*handler[4][256])(unsigned long);
	unsigned long          handler_data[4][256];

	struct device         *ddb_dev;
	u32                    ddb_dev_users;
	u32                    nr;
	u8                     iobuf[1028];

	u8                     leds;
	u32                    ts_irq;
	u32                    i2c_irq;

	struct mutex           mutex;

	u8                     tsbuf[TS_CAPTURE_LEN];
};

static inline void ddbwriteb(struct ddb *dev, u32 val, u32 adr)
{
	writeb(val, (char *) (dev->regs + (adr)));
}

static inline u32 ddbreadb(struct ddb *dev, u32 adr)
{
	return readb((char *) (dev->regs + (adr)));
}

static inline void ddbwritel0(struct ddb_link *link, u32 val, u32 adr)
{
	writel(val, (char *) (link->dev->regs + (adr)));
}

static inline u32 ddbreadl0(struct ddb_link *link, u32 adr)
{
	return readl((char *) (link->dev->regs + (adr)));
}

static inline void gtlw(struct ddb_link *link)
{
	while (1 & ddbreadl0(link, link->regs + 0x10))
		;
}

static u32 ddblreadl(struct ddb_link *link, u32 adr)
{
	if (unlikely(link->nr)) {
		unsigned long flags;
		u32 val;

		spin_lock_irqsave(&link->lock, flags);
		gtlw(link);
		ddbwritel0(link, adr & 0xfffc, link->regs + 0x14);
		ddbwritel0(link, 3, link->regs + 0x10);
		gtlw(link);
		val = ddbreadl0(link, link->regs + 0x1c);
		spin_unlock_irqrestore(&link->lock, flags);
		return val;
	}
	return readl((char *) (link->dev->regs + (adr)));
}

static void ddblwritel(struct ddb_link *link, u32 val, u32 adr)
{
	if (unlikely(link->nr)) {
		unsigned long flags;

		spin_lock_irqsave(&link->lock, flags);
		gtlw(link);
		ddbwritel0(link, 0xf0000 | (adr & 0xfffc), link->regs + 0x14);
		ddbwritel0(link, val, link->regs + 0x18);
		ddbwritel0(link, 1, link->regs + 0x10);
		spin_unlock_irqrestore(&link->lock, flags);
		return;
	}
	writel(val, (char *) (link->dev->regs + (adr)));
}

static u32 ddbreadl(struct ddb *dev, u32 adr)
{
	if (unlikely(adr & 0xf0000000)) {
		unsigned long flags;
		u32 val, l = (adr >> DDB_LINK_SHIFT);
		struct ddb_link *link = &dev->link[l];

		spin_lock_irqsave(&link->lock, flags);
		gtlw(link);
		ddbwritel0(link, adr & 0xfffc, link->regs + 0x14);
		ddbwritel0(link, 3, link->regs + 0x10);
		gtlw(link);
		val = ddbreadl0(link, link->regs + 0x1c);
		spin_unlock_irqrestore(&link->lock, flags);
		return val;
	}
	return readl((char *) (dev->regs + (adr)));
}

static void ddbwritel(struct ddb *dev, u32 val, u32 adr)
{
	if (unlikely(adr & 0xf0000000)) {
		unsigned long flags;
		u32 l = (adr >> DDB_LINK_SHIFT);
		struct ddb_link *link = &dev->link[l];

		spin_lock_irqsave(&link->lock, flags);
		gtlw(link);
		ddbwritel0(link, 0xf0000 | (adr & 0xfffc), link->regs + 0x14);
		ddbwritel0(link, val, link->regs + 0x18);
		ddbwritel0(link, 1, link->regs + 0x10);
		spin_unlock_irqrestore(&link->lock, flags);
		return;
	}
	writel(val, (char *) (dev->regs + (adr)));
}

static void gtlcpyto(struct ddb *dev, u32 adr, const u8 *buf,
		     unsigned int count)
{
	u32 val = 0, p = adr;
	u32 aa = p & 3;

	if (aa) {
		while (p & 3 && count) {
			val >>= 8;
			val |= *buf << 24;
			p++;
			buf++;
			count--;
		}
		ddbwritel(dev, val, adr);
	}
	while (count >= 4) {
		val = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
		ddbwritel(dev, val, p);
		p += 4;
		buf += 4;
		count -= 4;
	}
	if (count) {
		val = buf[0];
		if (count > 1)
			val |= buf[1] << 8;
		if (count > 2)
			val |= buf[2] << 16;
		ddbwritel(dev, val, p);
	}
}

static void gtlcpyfrom(struct ddb *dev, u8 *buf, u32 adr, long count)
{
	u32 val = 0, p = adr;
	u32 a = p & 3;

	if (a) {
		val = ddbreadl(dev, p) >> (8 * a);
		while (p & 3 && count) {
			*buf = val & 0xff;
			val >>= 8;
			p++;
			buf++;
			count--;
		}
	}
	while (count >= 4) {
		val = ddbreadl(dev, p);
		buf[0] = val & 0xff;
		buf[1] = (val >> 8) & 0xff;
		buf[2] = (val >> 16) & 0xff;
		buf[3] = (val >> 24) & 0xff;
		p += 4;
		buf += 4;
		count -= 4;
	}
	if (count) {
		val = ddbreadl(dev, p);
		buf[0] = val & 0xff;
		if (count > 1)
			buf[1] = (val >> 8) & 0xff;
		if (count > 2)
			buf[2] = (val >> 16) & 0xff;
	}
}

static void ddbcpyto(struct ddb *dev, u32 adr, void *src, long count)
{
	if (unlikely(adr & 0xf0000000))
		return gtlcpyto(dev, adr, src, count);
	return memcpy_toio((char *) (dev->regs + adr), src, count);
}

static void ddbcpyfrom(struct ddb *dev, void *dst, u32 adr, long count)
{
	if (unlikely(adr & 0xf0000000))
		return gtlcpyfrom(dev, dst, adr, count);
	return memcpy_fromio(dst, (char *) (dev->regs + adr), count);
}

#define ddbmemset(_dev, _adr, _val, _count) \
	memset_io((char *) (_dev->regs + (_adr)), (_val), (_count))


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

int ddbridge_flashread(struct ddb *dev, u32 link, u8 *buf, u32 addr, u32 len);

#define DDBRIDGE_VERSION "0.9.28+v7a-integrated"

#endif
