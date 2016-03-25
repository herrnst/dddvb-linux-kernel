/*
 * NXP TDA18212HN silicon tuner driver
 *
 * Copyright (C) 2011 Antti Palosaari <crope@iki.fi>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "tda18212.h"
#include <linux/regmap.h>

#define TDA18212_REG_ID_1		0x00
#define TDA18212_REG_POWER_STATE_1	0x05
#define TDA18212_REG_POWER_STATE_2	0x06
#define TDA18212_REG_IRQ_STATUS		0x08
#define TDA18212_REG_IRQ_CLEAR		0x0A
#define TDA18212_REG_AGC1_1		0x0C
#define TDA18212_REG_AGCK_1		0x0E
#define TDA18212_REG_AGC5_1		0x11
#define TDA18212_REG_REFERENCE		0x14
#define TDA18212_REG_MSM_1		0x19
#define TDA18212_REG_MSM_2		0x1A
#define TDA18212_REG_PSM_1		0x1B
#define TDA18212_REG_FLO_MAX		0x1D
#define TDA18212_REG_AGC1_2		0x24
#define TDA18212_REG_RF_FILTER_3	0x2E
#define TDA18212_REG_CP_CURRENT		0x30
#define TDA18212_REG_POWER_2		0x36
#define TDA18212_REG_RFCAL_LOG_1	0x38
#define TDA18212_REG_RFCAL_LOG_12	0x43

#define TDA18212_MST_PSM_AGC1		0
#define TDA18212_MST_AGC1_6_15DB	1

#define TDA18212_SLV_PSM_AGC1		1
#define TDA18212_SLV_AGC1_6_15DB	0

#define TDA18212_NUMREGS		0x44

struct tda18212_dev {
	struct tda18212_config cfg;
	struct i2c_client *client;
	struct regmap *regmap;

	u32 if_frequency;

	u16 tda_id;
	bool is_master;
	unsigned int regs[TDA18212_NUMREGS];
};

static int tda18212_read_regs(struct tda18212_dev *dev)
{
	int i;

	for (i = 0; i < TDA18212_NUMREGS; i++)
		regmap_read(dev->regmap, i, &dev->regs[i]);

	return 0;
}

static int tda18212_updatereg(struct tda18212_dev *dev, u32 reg)
{
	return regmap_write(dev->regmap, reg, dev->regs[reg]);
}

static int tda18212_set_params(struct dvb_frontend *fe)
{
	struct tda18212_dev *dev = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, i;
	u32 if_khz;
	u8 buf[9];
	#define DVBT_6   0
	#define DVBT_7   1
	#define DVBT_8   2
	#define DVBT2_6  3
	#define DVBT2_7  4
	#define DVBT2_8  5
	#define DVBC_6   6
	#define DVBC_8   7
	#define ATSC_VSB 8
	#define ATSC_QAM 9
	static const u8 bw_params[][3] = {
		     /* reg:   0f    13    23 */
		[DVBT_6]  = { 0xb3, 0x20, 0x03 },
		[DVBT_7]  = { 0xb3, 0x31, 0x01 },
		[DVBT_8]  = { 0xb3, 0x22, 0x01 },
		[DVBT2_6] = { 0xbc, 0x20, 0x03 },
		[DVBT2_7] = { 0xbc, 0x72, 0x03 },
		[DVBT2_8] = { 0xbc, 0x22, 0x01 },
		[DVBC_6]  = { 0x92, 0x50, 0x03 },
		[DVBC_8]  = { 0x92, 0x53, 0x03 },
		[ATSC_VSB] = { 0x7d, 0x20, 0x63 },
		[ATSC_QAM] = { 0x7d, 0x20, 0x63 },
	};

	dev_dbg(&dev->client->dev,
			"delivery_system=%d frequency=%d bandwidth_hz=%d\n",
			c->delivery_system, c->frequency,
			c->bandwidth_hz);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open I2C-gate */

	switch (c->delivery_system) {
	case SYS_ATSC:
		if_khz = dev->cfg.if_atsc_vsb;
		i = ATSC_VSB;
		break;
	case SYS_DVBC_ANNEX_B:
		if_khz = dev->cfg.if_atsc_qam;
		i = ATSC_QAM;
		break;
	case SYS_DVBT:
		switch (c->bandwidth_hz) {
		case 6000000:
			if_khz = dev->cfg.if_dvbt_6;
			i = DVBT_6;
			break;
		case 7000000:
			if_khz = dev->cfg.if_dvbt_7;
			i = DVBT_7;
			break;
		case 8000000:
			if_khz = dev->cfg.if_dvbt_8;
			i = DVBT_8;
			break;
		default:
			ret = -EINVAL;
			goto error;
		}
		break;
	case SYS_DVBT2:
		switch (c->bandwidth_hz) {
		case 6000000:
			if_khz = dev->cfg.if_dvbt2_6;
			i = DVBT2_6;
			break;
		case 7000000:
			if_khz = dev->cfg.if_dvbt2_7;
			i = DVBT2_7;
			break;
		case 8000000:
			if_khz = dev->cfg.if_dvbt2_8;
			i = DVBT2_8;
			break;
		default:
			ret = -EINVAL;
			goto error;
		}
		break;
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_C:
		if_khz = dev->cfg.if_dvbc;
		i = DVBC_8;
		break;
	default:
		ret = -EINVAL;
		goto error;
	}

	ret = regmap_write(dev->regmap, 0x23, bw_params[i][2]);
	if (ret)
		goto error;

	ret = regmap_write(dev->regmap, 0x06, 0x00);
	if (ret)
		goto error;

	ret = regmap_write(dev->regmap, 0x0f, bw_params[i][0]);
	if (ret)
		goto error;

	buf[0] = 0x02;
	buf[1] = bw_params[i][1];
	buf[2] = 0x03; /* default value */
	buf[3] = DIV_ROUND_CLOSEST(if_khz, 50);
	buf[4] = ((c->frequency / 1000) >> 16) & 0xff;
	buf[5] = ((c->frequency / 1000) >>  8) & 0xff;
	buf[6] = ((c->frequency / 1000) >>  0) & 0xff;
	buf[7] = 0xc1;
	buf[8] = 0x01;
	ret = regmap_bulk_write(dev->regmap, 0x12, buf, sizeof(buf));
	if (ret)
		goto error;

	/* actual IF rounded as it is on register */
	dev->if_frequency = buf[3] * 50 * 1000;

exit:
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close I2C-gate */

	return ret;

error:
	dev_dbg(&dev->client->dev, "failed=%d\n", ret);
	goto exit;
}

static int tda18212_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct tda18212_dev *dev = fe->tuner_priv;

	*frequency = dev->if_frequency;

	return 0;
}

static int tda18212_wakeup(struct tda18212_dev *dev)
{
	dev->regs[TDA18212_REG_POWER_STATE_2] &= ~0x0F;
	tda18212_updatereg(dev, TDA18212_REG_POWER_STATE_2);

	dev->regs[TDA18212_REG_REFERENCE] |= 0x40;
	tda18212_updatereg(dev, TDA18212_REG_REFERENCE);

	return 0;
}

static int tda18212_standby(struct tda18212_dev *dev)
{
	tda18212_read_regs(dev);

	dev->regs[TDA18212_REG_REFERENCE] &= ~0x40;
	if (tda18212_updatereg(dev, TDA18212_REG_REFERENCE))
		return -EIO;

	dev->regs[TDA18212_REG_POWER_STATE_2] &= ~0x0F;
	dev->regs[TDA18212_REG_POWER_STATE_2] |=
		dev->is_master ? 0x08 : 0x0E;
	if (tda18212_updatereg(dev, TDA18212_REG_POWER_STATE_2))
		return -EIO;

	return 0;
}

static int tda18212_sleep(struct dvb_frontend *fe)
{
	struct tda18212_dev *dev = fe->tuner_priv;

	if (dev->cfg.init_flags & TDA18212_INIT_DDSTV) {
		dev_dbg(&dev->client->dev, "standby");
		tda18212_standby(dev);
	}

	return 0;
}

static int tda18212_waitirq(struct tda18212_dev *dev, int timeout, u8 flags)
{
	unsigned int irqstatus = 0;
	int remain = timeout;

	while (remain > 0) {
		regmap_read(dev->regmap,
			TDA18212_REG_IRQ_STATUS, &irqstatus);
		if (irqstatus & flags)
			return 0;

		remain--;
		usleep_range(10000, 12000);
	}

	return -ETIMEDOUT;
}

static int tda18212_startcalibrate(struct tda18212_dev *dev)
{
	/* RSSI CK = 31.25 kHz */
	dev->regs[TDA18212_REG_POWER_2] &= ~0x02;
	if (tda18212_updatereg(dev, TDA18212_REG_POWER_2))
		return -EIO;

	/* AGC1 Do Step = 2 */
	dev->regs[TDA18212_REG_AGC1_2] =
		(dev->regs[TDA18212_REG_AGC1_2] & ~0x60) | 0x40;
	if (tda18212_updatereg(dev, TDA18212_REG_AGC1_2))
		return -EIO;

	/* AGC2 Do Step = 1 */
	dev->regs[TDA18212_REG_RF_FILTER_3] =
		(dev->regs[TDA18212_REG_RF_FILTER_3] & ~0xC0) | 0x40;
	if (tda18212_updatereg(dev, TDA18212_REG_RF_FILTER_3))
		return -EIO;

	/* AGCs Assym Up Step = 3 */
	dev->regs[TDA18212_REG_AGCK_1] |= 0xC0;
	if (tda18212_updatereg(dev, TDA18212_REG_AGCK_1))
		return -EIO;

	/* AGCs Assym Do Step = 2 */
	dev->regs[TDA18212_REG_AGC5_1] =
		(dev->regs[TDA18212_REG_AGC5_1] & ~0x60) | 0x40;
	if (tda18212_updatereg(dev, TDA18212_REG_AGC5_1))
		return -EIO;

	/* Reset IRQ */
	dev->regs[TDA18212_REG_IRQ_CLEAR] |= 0x80;
	if (tda18212_updatereg(dev, TDA18212_REG_IRQ_CLEAR))
		return -EIO;

	/* Set Calibration, start MSM */
	dev->regs[TDA18212_REG_MSM_1] = 0x3B;
	dev->regs[TDA18212_REG_MSM_2] = 0x01;
	if (tda18212_updatereg(dev, TDA18212_REG_MSM_1))
		return -EIO;
	if (tda18212_updatereg(dev, TDA18212_REG_MSM_2))
		return -EIO;

	dev->regs[TDA18212_REG_MSM_2] = 0x00;

	return 0;
}

static int tda18212_finishcalibrate(struct tda18212_dev *dev)
{
	int ret = 0;
	int i;

	ret = tda18212_waitirq(dev, 150, 0x80);
	if(ret)
		return ret;

	dev->regs[TDA18212_REG_FLO_MAX] = 0x0A;
	if (tda18212_updatereg(dev, TDA18212_REG_FLO_MAX))
		return -EIO;

	dev->regs[TDA18212_REG_AGC1_1] &= ~0xC0;
	/* LTEnable */
	if (dev->is_master)
		dev->regs[TDA18212_REG_AGC1_1] |= 0x80;
	dev->regs[TDA18212_REG_AGC1_1] |=
		(dev->is_master ? TDA18212_MST_AGC1_6_15DB
		: TDA18212_SLV_AGC1_6_15DB) << 6;
	if (tda18212_updatereg(dev, TDA18212_REG_AGC1_1))
		return -EIO;

	dev->regs[TDA18212_REG_PSM_1] &= ~0xC0;
	dev->regs[TDA18212_REG_PSM_1] |=
		(dev->is_master ? TDA18212_MST_PSM_AGC1
		: TDA18212_SLV_PSM_AGC1) << 6;
	if (tda18212_updatereg(dev, TDA18212_REG_PSM_1))
		return -EIO;

	/* XTOUT = 3 */
	dev->regs[TDA18212_REG_REFERENCE] |= 0x03;
	if (tda18212_updatereg(dev, TDA18212_REG_REFERENCE))
		return -EIO;

	/* read out RFCAL_LOG regs */
	for (i = TDA18212_REG_RFCAL_LOG_1;
			i <= TDA18212_REG_RFCAL_LOG_12; i++)
		regmap_read(dev->regmap, i, &dev->regs[i]);

	return ret;
}

static int tda18212_read_chipid(struct regmap *regmap, u32 initflags,
		u8 *chipid)
{
	unsigned int id;
	int ret;

	if (initflags & TDA18212_INIT_DDSTV)
		return regmap_raw_read(regmap, TDA18212_REG_ID_1, chipid, 2);

	ret = regmap_read(regmap, TDA18212_REG_ID_1, &id);
	chipid[0] = id & 0xff;
	return ret;
}

static int tda18212_init_extended(struct tda18212_dev *dev)
{
	unsigned int powerstate = 0;
	int ret = 0;

	ret = regmap_read(dev->regmap, TDA18212_REG_POWER_STATE_1,
		&powerstate);
	if (ret < 0)
		return ret;

	if (dev->is_master) {
		if (powerstate & 0x02)
			ret = tda18212_waitirq(dev, 10, 0x20);

	} else {
		regmap_write(dev->regmap, TDA18212_REG_FLO_MAX, 0x00);
		regmap_write(dev->regmap, TDA18212_REG_CP_CURRENT, 0x68);
	}

	tda18212_read_regs(dev);

	tda18212_wakeup(dev);
	tda18212_startcalibrate(dev);
	tda18212_finishcalibrate(dev);
	tda18212_standby(dev);

	return ret;
}

static const struct dvb_tuner_ops tda18212_tuner_ops = {
	.info = {
		.name           = "NXP TDA18212",

		.frequency_min  =  48000000,
		.frequency_max  = 864000000,
		.frequency_step =      1000,
	},

	.sleep			= tda18212_sleep,
	.set_params		= tda18212_set_params,
	.get_if_frequency	= tda18212_get_if_frequency,
};

static int tda18212_probe_do(struct i2c_client *client,
	const struct i2c_device_id *id, struct tda18212_config *cfg,
	struct dvb_frontend *fe)
{
	struct tda18212_dev *dev;
	int ret;
	u8 chipid[2] = { 0, 0 };
	static const struct regmap_config regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "kzalloc() failed\n");
		goto err;
	}

	memcpy(&dev->cfg, cfg, sizeof(struct tda18212_config));
	dev->client = client;
	dev->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(dev->regmap)) {
		ret = PTR_ERR(dev->regmap);
		goto err;
	}

	ret = tda18212_read_chipid(dev->regmap, cfg->init_flags, chipid);

	/* retry probe if desired */
	if (ret && (cfg->init_flags & TDA18212_INIT_RETRY))
		ret = tda18212_read_chipid(dev->regmap, cfg->init_flags, chipid);

	dev_dbg(&dev->client->dev, "chip_id=%02x\n", chipid[0]);

	if (ret)
		goto err;

	dev->tda_id = ((chipid[0] & 0x7f) << 8) | chipid[1];
	dev->is_master = ((chipid[0] & 0x80) != 0);

	if (!(dev->tda_id == 18212 || dev->tda_id == 18176)) {
		ret = -ENODEV;
		goto err;
	}

	if (cfg->init_flags & TDA18212_INIT_DDSTV) {
		ret = tda18212_init_extended(dev);
		if (ret)
			goto err;
	}

	dev_info(&dev->client->dev,
			"NXP TDA18212HN/%s successfully identified\n",
			(dev->is_master ? "M" : "S"));

	fe->tuner_priv = dev;
	memcpy(&fe->ops.tuner_ops, &tda18212_tuner_ops,
			sizeof(struct dvb_tuner_ops));
	i2c_set_clientdata(client, dev);

	return 0;
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	kfree(dev);
	return ret;
}

static int tda18212_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct tda18212_config *cfg = client->dev.platform_data;
	struct dvb_frontend *fe = cfg->fe;
	int ret;

	/* check if the tuner is there */
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open I2C-gate */

	ret = tda18212_probe_do(client, id, cfg, fe);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close I2C-gate */

	return ret;
}

static int tda18212_remove(struct i2c_client *client)
{
	struct tda18212_dev *dev = i2c_get_clientdata(client);
	struct dvb_frontend *fe = dev->cfg.fe;

	dev_dbg(&client->dev, "\n");

	memset(&fe->ops.tuner_ops, 0, sizeof(struct dvb_tuner_ops));
	fe->tuner_priv = NULL;
	kfree(dev);

	return 0;
}

static const struct i2c_device_id tda18212_id[] = {
	{"tda18212", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tda18212_id);

static struct i2c_driver tda18212_driver = {
	.driver = {
		.name	= "tda18212",
	},
	.probe		= tda18212_probe,
	.remove		= tda18212_remove,
	.id_table	= tda18212_id,
};

module_i2c_driver(tda18212_driver);

MODULE_DESCRIPTION("NXP TDA18212HN silicon tuner driver");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_LICENSE("GPL");
