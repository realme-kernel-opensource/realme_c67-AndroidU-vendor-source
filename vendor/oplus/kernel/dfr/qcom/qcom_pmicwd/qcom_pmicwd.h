/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#ifndef __QCOM_PMICWD_H__
#define __QCOM_PMICWD_H__

#include <linux/regmap.h>
#include <linux/input/qpnp-power-on.h>

struct pmicwd_desc {
        struct qpnp_pon    *pon;
        struct task_struct *wd_task;
        struct mutex       wd_task_mutex;
        unsigned int       pmicwd_state;/* |reserver|rst type|timeout|enable| */
        u8                 suspend_state;/* record the suspend state */
};

struct qpnp_pon {
        struct device           *dev;
        struct regmap           *regmap;
        struct input_dev        *pon_input;
        struct qpnp_pon_config  *pon_cfg;
        struct pon_regulator    *pon_reg_cfg;
        struct list_head        restore_regs;
        struct list_head        list;
        struct mutex            restore_lock;
        struct delayed_work     bark_work;
        struct dentry           *debugfs;
        u16                     base;
        u16                     pbs_base;
        u8                      subtype;
        u8                      pon_ver;
        u8                      warm_reset_reason1;
        u8                      warm_reset_reason2;
        int                     num_pon_config;
        int                     num_pon_reg;
        int                     pon_trigger_reason;
        int                     pon_power_off_reason;
        u32                     dbc_time_us;
        u32                     uvlo;
        int                     warm_reset_poff_type;
        int                     hard_reset_poff_type;
        int                     shutdown_poff_type;
        int                     resin_warm_reset_type;
        int                     resin_hard_reset_type;
        int                     resin_shutdown_type;
        bool                    is_spon;
        bool                    store_hard_reset_reason;
        bool                    resin_hard_reset_disable;
        bool                    resin_shutdown_disable;
        bool                    ps_hold_hard_reset_disable;
        bool                    ps_hold_shutdown_disable;
        bool                    kpdpwr_dbc_enable;
        bool                    resin_pon_reset;
        ktime_t                 kpdpwr_last_release_time;
        bool                    legacy_hard_reset_offset;
};

#define PWD_TAG "[PMICWD]"
#define PWD_INFO(fmt, ...) printk(KERN_INFO PWD_TAG pr_fmt(fmt), ##__VA_ARGS__)
#define PWD_WARN(fmt, ...) printk(KERN_WARNING PWD_TAG pr_fmt(fmt), ##__VA_ARGS__)
#define PWD_ERR(fmt, ...) printk(KERN_ERR PWD_TAG pr_fmt(fmt), ##__VA_ARGS__)

#undef ASSERT
#define ASSERT(x) BUG_ON(!(x))
static inline int dup_qpnp_pon_masked_write(struct qpnp_pon *pon, u16 addr, u8 mask, u8 val)
{
        int rc;

        rc = regmap_update_bits(pon->regmap, addr, mask, val);
        if (rc) {
                PWD_ERR("Register write failed, addr=0x%04X, rc=%d\n", addr, rc);
        }

        return rc;
}

void kpdpwr_init(void);
void pmicwd_init(struct platform_device *pdev);
int qpnp_pon_wd_pet(struct qpnp_pon *pon);

#endif
