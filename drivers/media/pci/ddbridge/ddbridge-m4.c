// SPDX-License-Identifier: GPL-2.0
/*
 * ddbridge-m4.c: Digital Devices MAX M4 driver
 *
 * Copyright (C) 2018 Digital Devices GmbH
 *                    Marcus Metzler <mocm@metzlerbros.de>
 *                    Ralph Metzler <rjkm@metzlerbros.de>
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

#include "ddbridge.h"
#include "ddbridge-io.h"
#include "ddbridge-mci.h"

struct m4_base {
	struct mci_base      mci_base;

};

struct m4 {
	struct mci           mci;

};

static struct dvb_frontend_ops m4_ops = {
	/* TODO: add/enable SYS_DVBC2 */
	.delsys = { SYS_DVBC_ANNEX_A, SYS_DVBT, SYS_DVBT2, SYS_ISDBT,
		    SYS_DVBS, SYS_DVBS2, },
	.info = {
		.name                = "Digital Devices MaxM4 MCI",
		.frequency_min       = 47000000, /* DVB-T: 47125000 */
		.frequency_max       = 865000000, /* DVB-C: 862000000 */
		.symbol_rate_min     = 870000,
		.symbol_rate_max     = 11700000,
		.frequency_stepsize  = 0,
		.frequency_tolerance = 0,
		.caps = FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_32 |
		        FE_CAN_QAM_64 | FE_CAN_QAM_128 | FE_CAN_QAM_256 |
		        FE_CAN_QAM_AUTO |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO | FE_CAN_HIERARCHY_AUTO |
			FE_CAN_RECOVER | FE_CAN_MUTE_TS | FE_CAN_2G_MODULATION
	},
};

const struct mci_cfg ddb_max_m4_cfg = {
	.type = 0,
	.fe_ops = &m4_ops,
	.base_size = sizeof(struct m4_base),
	.state_size = sizeof(struct m4),
};
