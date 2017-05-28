/*
 * ddbridge.h: Digital Devices PCIe bridge driver
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

#ifndef _DDBRIDGE_H_
#define _DDBRIDGE_H_

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <asm/dma.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/ca.h>
#include <linux/socket.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_ringbuffer.h"
#include "dvb_ca_en50221.h"
#include "dvb_net.h"
#include "cxd2099.h"

#define DDB_MAX_I2C     4
#define DDB_MAX_PORT    10
#define DDB_MAX_INPUT   8
#define DDB_MAX_OUTPUT  10
#define DDB_MAX_LINK    4
#define DDB_LINK_SHIFT  28

#define DDB_LINK_TAG(_x) (_x << DDB_LINK_SHIFT)

#define DDB_XO2_TYPE_NONE	0
#define DDB_XO2_TYPE_DUOFLEX	1
#define DDB_XO2_TYPE_CI		2

struct ddb_info {
	int   type;
#define DDB_NONE		0
#define DDB_OCTOPUS		1
#define DDB_OCTOPUS_CI		2
#define DDB_OCTOPUS_MAX_CT	6
	char *name;
	int   port_num;
	int   i2c_num;
	int   led_num;
	int   fan_num;
	int   temp_num;
	u32   board_control;
	u32   board_control_2;
	u8    ts_quirks;
#define TS_QUIRK_SERIAL   1
#define TS_QUIRK_REVERSED 2
#define TS_QUIRK_ALT_OSC  8
	int   temp_bus;
};

/* DMA_SIZE MUST be smaller than 256k and
   MUST be divisible by 188 and 128 !!! */

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
	u32                    nr;
	dma_addr_t             pbuf[DMA_MAX_BUFS];
	u8                    *vbuf[DMA_MAX_BUFS];
	u32                    num;
	u32                    size;
	u32                    div;
	u32                    bufreg;

	struct tasklet_struct  tasklet;
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
	int (*gate_ctrl)(struct dvb_frontend *, int);
	int                    attached;
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
	struct ddb_dma        *dma;
	struct ddb_io         *redirect;
	struct ddb_io         *redo;
	struct ddb_io         *redi;
};

#define ddb_output ddb_io
#define ddb_input  ddb_io

struct ddb_i2c {
	struct ddb            *dev;
	u32                    nr;
	struct i2c_adapter     adap;
	u32                    regs;
	u32                    rbuf;
	u32                    wbuf;
	int                    done;
	wait_queue_head_t      wq;
};

struct ddb_port {
	struct ddb            *dev;
	u32                    nr;
	struct ddb_i2c        *i2c;
	struct mutex           i2c_gate_lock;
	u32                    class;
#define DDB_PORT_NONE           0
#define DDB_PORT_CI             1
#define DDB_PORT_TUNER          2
#define DDB_PORT_LOOP           3
	char                  *type_name;
	u32                    type;
#define DDB_TUNER_NONE			0
#define DDB_TUNER_DVBS_ST		1
#define DDB_TUNER_DVBS_ST_AA		2
#define DDB_TUNER_DVBCT_TR		3
#define DDB_TUNER_DVBCT_ST		4
#define DDB_CI_INTERNAL			5
#define DDB_CI_EXTERNAL_SONY		6
#define DDB_TUNER_DVBCT2_SONY_P		7
#define DDB_TUNER_DVBC2T2_SONY_P	8
#define DDB_TUNER_ISDBT_SONY_P		9
#define DDB_TUNER_DVBS_STV0910_P	10
#define DDB_TUNER_DVBS_STV0910_PR	14
#define DDB_TUNER_DVBC2T2I_SONY_P	15
#define DDB_TUNER_XO2_DVBS_STV0910	32
#define DDB_TUNER_XO2_DVBCT2_SONY	33
#define DDB_TUNER_XO2_ISDBT_SONY	34
#define DDB_TUNER_XO2_DVBC2T2_SONY	35
#define DDB_TUNER_XO2_ATSC_ST		36
#define DDB_TUNER_XO2_DVBC2T2I_SONY	37

	u32                    adr;

	struct ddb_input      *input[2];
	struct ddb_output     *output;
	struct dvb_ca_en50221 *en;
	struct ddb_dvb         dvb[2];
	u32                    gap;
};

struct ddb {
	struct pci_dev             *pdev;
	const struct pci_device_id *id;
	struct ddb_info            *info;
	int                         msi;
	unsigned char __iomem      *regs;
	struct ddb_port             port[DDB_MAX_PORT];
	struct ddb_i2c              i2c[DDB_MAX_I2C];
	struct ddb_input            input[DDB_MAX_INPUT];
	struct ddb_output           output[DDB_MAX_OUTPUT];
	struct dvb_adapter          adap[DDB_MAX_INPUT];
	struct ddb_dma              dma[DDB_MAX_INPUT + DDB_MAX_OUTPUT];

	void                        (*handler[20])(unsigned long);
	unsigned long               handler_data[20];

	struct device              *ddb_dev;
	u32                         nr;
	u8                          iobuf[1028];

	u8                          leds;
	u32                         ts_irq;
	u32                         i2c_irq;

	u32                         hwid;
	u32                         regmap;
};

/****************************************************************************/

#endif
