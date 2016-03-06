#ifndef _STV0367DD_H_
#define _STV0367DD_H_

#include <linux/types.h>
#include <linux/i2c.h>

struct stv0367dd_cfg {
	u8  adr;
	u32 xtal;
	u8 parallel;
	u8 cont_clock;
};

#if IS_REACHABLE(CONFIG_DVB_STV0367DD)

extern struct dvb_frontend *stv0367dd_attach(struct i2c_adapter *i2c,
					     struct stv0367dd_cfg *cfg,
					     struct dvb_frontend **fe_t);

#else

static inline struct dvb_frontend *stv0367dd_attach(struct i2c_adapter *i2c,
	struct stv0367dd_cfg *cfg, struct dvb_frontend **fe_t)
{
	pr_warn("%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif /* CONFIG_DVB_STV0367DD */

#endif /* _STV0367DD_H_ */
