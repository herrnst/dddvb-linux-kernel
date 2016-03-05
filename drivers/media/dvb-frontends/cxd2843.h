#ifndef _CXD2843_H_
#define _CXD2843_H_

#include <linux/types.h>
#include <linux/i2c.h>

struct cxd2843_cfg {
	u8  adr;
	u32 ts_clock;
	u8  parallel;
};

#if IS_REACHABLE(CONFIG_DVB_CXD2843)

extern struct dvb_frontend *cxd2843_attach(struct i2c_adapter *i2c,
					   struct cxd2843_cfg *cfg);


#else

static inline struct dvb_frontend *cxd2843_attach(struct i2c_adapter *i2c,
					   struct cxd2843_cfg *cfg)
{
	pr_warn("%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif /* CONFIG_DVB_CXD2843 */

#endif /* _CXD2843_H_ */
