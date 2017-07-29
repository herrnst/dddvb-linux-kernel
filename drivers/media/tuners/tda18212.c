// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * NXP TDA18212HN silicon tuner driver
 *
 * Copyright (C) 2011 Antti Palosaari <crope@iki.fi>
 */

#include "tda18212.h"
#include <linux/regmap.h>

struct tda18212_dev {
	struct tda18212_config cfg;
	struct i2c_client *client;
	struct regmap *regmap;

	u32 if_frequency;
};

#define	TDA18212_ID_1			0x00
#define	TDA18212_ID_2			0x01
#define	TDA18212_ID_3			0x02
#define	TDA18212_THERMO_1		0x03
#define	TDA18212_THERMO_2		0x04
#define	TDA18212_POWER_STATE_1		0x05
#define	TDA18212_POWER_STATE_2		0x06
#define	TDA18212_INPUT_POWER_LEVEL	0x07
#define	TDA18212_IRQ_STATUS		0x08
#define	TDA18212_IRQ_ENABLE		0x09
#define	TDA18212_IRQ_CLEAR		0x0A
#define	TDA18212_IRQ_SET		0x0B
#define	TDA18212_AGC1_1			0x0C
#define	TDA18212_AGC2_1			0x0D
#define	TDA18212_AGCK_1			0x0E
#define	TDA18212_RF_AGC_1		0x0F
#define	TDA18212_IR_MIXER_1		0x10
#define	TDA18212_AGC5_1			0x11
#define	TDA18212_IF_AGC			0x12
#define	TDA18212_IF_1			0x13
#define	TDA18212_REFERENCE		0x14
#define	TDA18212_IF_FREQUENCY_1		0x15
#define	TDA18212_RF_FREQUENCY_1		0x16
#define	TDA18212_RF_FREQUENCY_2		0x17
#define	TDA18212_RF_FREQUENCY_3		0x18
#define	TDA18212_MSM_1			0x19
#define	TDA18212_MSM_2			0x1A
#define	TDA18212_PSM_1			0x1B
#define	TDA18212_DCC_1			0x1C
#define	TDA18212_FLO_MAX		0x1D
#define	TDA18212_IR_CAL_1		0x1E
#define	TDA18212_IR_CAL_2		0x1F
#define	TDA18212_IR_CAL_3		0x20
#define	TDA18212_IR_CAL_4		0x21
#define	TDA18212_VSYNC_MGT		0x22
#define	TDA18212_IR_MIXER_2		0x23
#define	TDA18212_AGC1_2			0x24
#define	TDA18212_AGC5_2			0x25
#define	TDA18212_RF_CAL_1		0x26
#define	TDA18212_RF_CAL_2		0x27
#define	TDA18212_RF_CAL_3		0x28
#define	TDA18212_RF_CAL_4		0x29
#define	TDA18212_RF_CAL_5		0x2A
#define	TDA18212_RF_CAL_6		0x2B
#define	TDA18212_RF_FILTER_1		0x2C
#define	TDA18212_RF_FILTER_2		0x2D
#define	TDA18212_RF_FILTER_3		0x2E
#define	TDA18212_RF_BAND_PASS_FILTER	0x2F
#define	TDA18212_CP_CURRENT		0x30
#define	TDA18212_AGC_DET_OUT		0x31
#define	TDA18212_RF_AGC_GAIN_1		0x32
#define	TDA18212_RF_AGC_GAIN_2		0x33
#define	TDA18212_IF_AGC_GAIN		0x34
#define	TDA18212_POWER_1		0x35
#define	TDA18212_POWER_2		0x36
#define	TDA18212_MISC_1			0x37
#define	TDA18212_RFCAL_LOG_1		0x38
#define	TDA18212_RFCAL_LOG_2		0x39
#define	TDA18212_RFCAL_LOG_3		0x3A
#define	TDA18212_RFCAL_LOG_4		0x3B
#define	TDA18212_RFCAL_LOG_5		0x3C
#define	TDA18212_RFCAL_LOG_6		0x3D
#define	TDA18212_RFCAL_LOG_7		0x3E
#define	TDA18212_RFCAL_LOG_8		0x3F
#define	TDA18212_RFCAL_LOG_9		0x40
#define	TDA18212_RFCAL_LOG_10		0x41
#define	TDA18212_RFCAL_LOG_11		0x42
#define	TDA18212_RFCAL_LOG_12		0x43

#define	TDA18212_REG_MAX		0x44

static int tda18212_wait_irq(struct tda18212_dev *dev, u8 mask, int timeout)
{
	unsigned int irq = 0;
	int ret;

	while (1) {
		/* read IRQ_STATUS */
		ret = regmap_read(dev->regmap, TDA18212_IRQ_STATUS, &irq);
		if (ret)
			return -1;

		/* break if bit indicated by mask is set */
		if (irq & mask)
			break;

		timeout--;
		if (!timeout)
			return -1;

		/* wait ~10ms */
		usleep_range(10000, 12000);
	}

	return 0;
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
		/* regs: RF_AGC_1 / IF_1 / IR_MIXER_2 */
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

	ret = regmap_write(dev->regmap, TDA18212_IR_MIXER_2, bw_params[i][2]);
	if (ret)
		goto error;

	ret = regmap_write(dev->regmap, TDA18212_POWER_STATE_2, 0x00);
	if (ret)
		goto error;

	ret = regmap_write(dev->regmap, TDA18212_RF_AGC_1, bw_params[i][0]);
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
	ret = regmap_bulk_write(dev->regmap, TDA18212_IF_AGC, buf, sizeof(buf));
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

static int tda18212_get_rf_strength(struct dvb_frontend *fe, u16 *str)
{
	struct tda18212_dev *dev = fe->tuner_priv;
	int ret;
	unsigned int irq = 0, pwr = 0;

        /* open I2C gate */
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = regmap_read(dev->regmap, TDA18212_IRQ_CLEAR, &irq);
	if (ret)
		goto exit;

	irq |= 0x80; /* reset IRQ */
	ret = regmap_write(dev->regmap, TDA18212_IRQ_CLEAR, irq);
	if (ret)
		goto exit;

	/* power measurement */
	ret = regmap_write(dev->regmap, TDA18212_MSM_1, 0x80);
	if (ret)
		goto exit;
	/* start MSM */
	ret = regmap_write(dev->regmap, TDA18212_MSM_2, 0x01);
	if (ret)
		goto exit;

	/* wait for IRQ (until done) up to 700ms */
	ret = tda18212_wait_irq(dev, 0x80, 70);
	if (ret)
		goto exit;

	ret = regmap_read(dev->regmap, TDA18212_INPUT_POWER_LEVEL, &pwr);
	if (ret)
		goto exit;

	pwr &= 0x7f;

exit:
	/* close I2C gate */
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	*str = pwr;

	return ret;
}

static int tda18212_get_if_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct tda18212_dev *dev = fe->tuner_priv;

	*frequency = dev->if_frequency;

	return 0;
}

static const struct dvb_tuner_ops tda18212_tuner_ops = {
	.info = {
		.name              = "NXP TDA18212",

		.frequency_min_hz  =  48 * MHz,
		.frequency_max_hz  = 864 * MHz,
		.frequency_step_hz =   1 * kHz,
	},

	.set_params    = tda18212_set_params,
	.get_if_frequency = tda18212_get_if_frequency,
	.get_rf_strength = tda18212_get_rf_strength,
};

static int tda18212_probe(struct i2c_client *client)
{
	struct tda18212_config *cfg = client->dev.platform_data;
	struct dvb_frontend *fe = cfg->fe;
	struct tda18212_dev *dev;
	int ret;
	unsigned int chip_id;
	char *version;
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

	/* check if the tuner is there */
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open I2C-gate */

	ret = regmap_read(dev->regmap, TDA18212_ID_1, &chip_id);
	dev_dbg(&dev->client->dev, "chip_id=%02x\n", chip_id);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close I2C-gate */

	if (ret)
		goto err;

	switch (chip_id) {
	case 0xc7:
		version = "M"; /* master */
		break;
	case 0x47:
		version = "S"; /* slave */
		break;
	default:
		ret = -ENODEV;
		goto err;
	}

	dev_info(&dev->client->dev,
			"NXP TDA18212HN/%s successfully identified\n", version);

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

static void tda18212_remove(struct i2c_client *client)
{
	struct tda18212_dev *dev = i2c_get_clientdata(client);
	struct dvb_frontend *fe = dev->cfg.fe;

	dev_dbg(&client->dev, "\n");

	memset(&fe->ops.tuner_ops, 0, sizeof(struct dvb_tuner_ops));
	fe->tuner_priv = NULL;
	kfree(dev);
}

static const struct i2c_device_id tda18212_id[] = {
	{ "tda18212" },
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
