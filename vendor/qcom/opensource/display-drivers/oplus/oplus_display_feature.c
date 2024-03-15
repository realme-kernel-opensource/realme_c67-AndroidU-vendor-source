/***************************************************************
  ** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
  **
  ** File : oplus_display_feature.c
  ** Description : oplus display char dev  /dev/oplus_panel
  ** Version : 1.0
  ** Date : 2023/08/23
  ** Author : Display
******************************************************************/
#include <linux/of.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pwm.h>

#include "dsi_parser.h"
#include "dsi_panel.h"
#include "dsi_defs.h"

#include "oplus_display_feature.h"

static struct kobject *oplus_display_feature_kobj;
static struct oplus_display_feature *g_feature = NULL;
#define OPLUS_ATTR(_name, _mode, _show, _store) \
    struct kobj_attribute oplus_attr_##_name = __ATTR(_name, _mode, _show, _store)

static int oplus_backlight_remapping(int brightness)
{
	int bl_lvl = bl_mapping_table[brightness];
	return bl_lvl;
}

static int oplus_display_pwm_update_state(int duty_us)
{
    int rc = 0;
    u32 duty_ns = 0, duty_us_backup = 0;
    u32 period_ns = 0;
//    pr_info("%s: div_lvl = %u\n", __func__, duty_us);
    if (!g_feature->pwm_dev) {
        pr_err("%s: pwm device is not defined\n", __func__);
        return -EINVAL;
    }
    duty_us_backup = duty_us;
    if(duty_us <= g_feature->dimming2_mapping_size)
		duty_us = g_feature->dimming2_mapping[duty_us];
	else
		duty_us = 0;
	if (duty_us > g_feature->pwm_period_us) {
        pr_err("%s: div_lvl Out of range\n", __func__);
        return -EINVAL;
    }
//    pr_info("[drm] duty_us_backup:%d, duty_us=%d\n", duty_us_backup, duty_us);
    g_feature->pwm_duty_us = duty_us; 
    period_ns = g_feature->pwm_period_us * NSEC_PER_USEC;
    duty_ns = duty_us * NSEC_PER_USEC;
    rc = pwm_config(g_feature->pwm_dev, duty_ns, period_ns);
    if (rc) {
        pr_err("%s: failed to change pwm config, rc=%d\n", __func__, rc);
        goto error;
    }
    if (duty_us == 0) {
        pwm_disable(g_feature->pwm_dev);
    } else {
        pwm_enable(g_feature->pwm_dev);
    }
    return 0;
error:
    return rc;
}

static int opls_pwm_div_update_status(int div_lvl)
{
	int rc = 0;

	if (g_feature->oplus_display_pwm_enable) {
		rc = oplus_display_pwm_update_state(div_lvl);
		pr_info("%s: div_lvl = %d\n", __func__, div_lvl);
		if (rc)
			pr_err("%s: failed rc=%d\n", __func__, rc);
	}
	return rc;
}

static int oplus_display_pwm_init(struct dsi_panel *panel)
{
    int rc = 0;
    struct dsi_parser_utils *utils = &panel->utils;
    g_feature->oplus_display_pwm_enable = utils->read_bool(utils->data, "oplus,display-pwm-feature");
    pr_err("oplus,display-pwm-feature: %s", g_feature->oplus_display_pwm_enable ? "true" : "false");
    if (g_feature->oplus_display_pwm_enable) {
        g_feature->oplus_display_pwm_update = opls_pwm_div_update_status;
        g_feature->pwm_dev = devm_of_pwm_get(panel->parent, panel->panel_of_node, NULL);
        if (IS_ERR_OR_NULL(g_feature->pwm_dev)) {
            rc = PTR_ERR(g_feature->pwm_dev);
            if (rc != -EPROBE_DEFER)
                pr_err("%s: Get pwm device for failed, rc=%d\n", __func__, rc);
            goto error;
        }
        rc = utils->read_u32(utils->data, "oplus,pwm-period-us", &g_feature->pwm_period_us);
        if (rc) {
            pr_err("pwm-period-us is not defined, rc=%d\n", rc);
            goto error;
        }
        rc = utils->read_u32(utils->data, "oplus,dimming2-mapping-size", &g_feature->dimming2_mapping_size);
        if (rc) {
            pr_err("oplus,dimming2-mapping-size is not defined, rc=%d\n", rc);
            goto error;
        }
        rc = utils->read_u32_array(utils->data, "oplus,dimming2-mapping", g_feature->dimming2_mapping, g_feature->dimming2_mapping_size);
        if (rc) {
            pr_err("oplus,dimming2-mapping is not defined, rc=%d\n", rc);
            goto error;
        }
        g_feature->pwm_pin = devm_pinctrl_get(panel->parent);
        if (IS_ERR_OR_NULL(g_feature->pwm_pin)) {
            rc = PTR_ERR(g_feature->pwm_pin);
            pr_err("%s: Target does not use pinctrl %d\n", __func__, rc);
            goto error;
        }
        /* find pinctrl state by name */
        g_feature->pwm_pin_state = pinctrl_lookup_state(g_feature->pwm_pin, "pwm_pin");
        if (IS_ERR_OR_NULL(g_feature->pwm_pin_state)) {
            rc = PTR_ERR(g_feature->pwm_pin_state);
            pr_err("%s: Can not lookup pwm_pin pinstate %d\n", __func__, rc);
            goto error;
        }
        /* select the pinctrl state */
        rc = pinctrl_select_state(g_feature->pwm_pin, g_feature->pwm_pin_state);
        if (rc < 0)
            pr_err("%s: Cannot get active pinctrl state\n", __func__);
    }
    return 0;
error:
    return rc;
}

ssize_t oplus_display_set_pwm_attr(struct kobject *obj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    int rc = 0;
	int duty_us;
	int period_us;
	int test_flag;
	pr_info("%s Entry\n", __func__);
    if (!g_feature->oplus_display_pwm_enable) {
        return count;
    }
	rc = sscanf(buf, "%d:%d:%d",&test_flag, &duty_us, &period_us);
	pr_info("%s sscanf ret=%d\n", __func__, rc);
	if (rc != 3)
		return -EINVAL;
	pr_err("%s: test_flag=%d : duty_us=%d : period_us=%d\n", __func__, test_flag, duty_us, period_us);
	if (g_feature->pwm_period_old_us == 0){
		g_feature->pwm_period_old_us = g_feature->pwm_period_us;
	}
	if (test_flag){
		g_feature->pwm_period_us = period_us;
	}else{
		g_feature->pwm_period_us = g_feature->pwm_period_old_us;
		g_feature->pwm_period_old_us = 0;
	}
	rc = oplus_display_pwm_update_state(duty_us);
	return (rc < 0) ? rc : count;
}
ssize_t oplus_display_get_pwm_attr(struct kobject *obj, struct kobj_attribute *attr, char *buf)
{  
	pr_info("%s Entry\n", __func__);
    if (g_feature->oplus_display_pwm_enable) {
	    return scnprintf(buf, PAGE_SIZE, "%d:%d\n", g_feature->pwm_duty_us, g_feature->pwm_period_us);
    } else {
        return scnprintf(buf, PAGE_SIZE, "%s\n", "no support pwm func.");
    }
}

struct oplus_display_feature *get_oplus_display_feature(void) {
    return g_feature;
}
EXPORT_SYMBOL(get_oplus_display_feature);

static OPLUS_ATTR(pwm, S_IRUGO | S_IWUSR, oplus_display_get_pwm_attr, oplus_display_set_pwm_attr);

static struct attribute *oplus_display_feature_attrs[] = {
    &oplus_attr_pwm.attr,
    NULL,
};

static struct attribute_group oplus_display_feature_attr_group = {
    .attrs = oplus_display_feature_attrs,
};

int oplus_display_feature_init(struct dsi_display *display)
{
    int ret = 0;
    struct dsi_panel *panel;
    if (!display && !display->panel) {
        return -EPROBE_DEFER;
    }
    panel = display->panel;
    g_feature = kzalloc(sizeof(struct oplus_display_feature), GFP_KERNEL);
    if (!g_feature) {
        pr_err("%s kzalloc fail\n", __func__);
        return -ENOMEM;
    }
	g_feature->oplus_bl_maping = oplus_backlight_remapping;
    ret = oplus_display_pwm_init(panel);
    if (ret < 0) {
        pr_err("%s  oplus display pwm init fail!\n", __func__);
    }
    oplus_display_feature_kobj = kobject_create_and_add("oplus_display_feature", NULL);
    if (!oplus_display_feature_kobj) {
        pr_err("%s kobject_create_and_add fail\n", __func__);
        ret = -ENOMEM;
        goto err_free_disp_feature;
    }
    ret = sysfs_create_group(oplus_display_feature_kobj, &oplus_display_feature_attr_group);
    if (ret) {
        goto error_remove_kobj;
    }
    ret = sysfs_create_link(oplus_display_feature_kobj, &display->pdev->dev.kobj, "panel");
    if (ret) {
        goto error_remove_sysfs_group;
    }
    return 0;
error_remove_sysfs_group:
    sysfs_remove_group(oplus_display_feature_kobj, &oplus_display_feature_attr_group);
error_remove_kobj:
    kobject_put(oplus_display_feature_kobj);
    oplus_display_feature_kobj = NULL;
err_free_disp_feature:
    kfree(g_feature);
    return ret;
}
EXPORT_SYMBOL(oplus_display_feature_init);

void  oplus_display_feature_exit(void)
{
    sysfs_remove_link(oplus_display_feature_kobj, "panel");
    sysfs_remove_group(oplus_display_feature_kobj, &oplus_display_feature_attr_group);
    kobject_put(oplus_display_feature_kobj);
    kfree(g_feature);
}
EXPORT_SYMBOL(oplus_display_feature_exit);

