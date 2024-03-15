// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/soc/qcom/smem.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <soc/oplus/boot/boot_mode.h>
#include <linux/printk.h>
#if IS_ENABLED(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM)

#define MAX_CMD_LENGTH 32

static struct kobject *systeminfo_kobj;
static int ftm_mode = MSM_BOOT_MODE__NORMAL;

#if IS_MODULE(CONFIG_OPLUS_FEATURE_PROJECTINFO)
extern char oplus_ftm_mode[];
extern char startup_mode[];
extern char charger_present[];
extern char bootmode[];
#endif

int __init  board_ftm_mode_init(void)
{
#if IS_MODULE(CONFIG_OPLUS_FEATURE_PROJECTINFO)
#if IS_MODULE(CONFIG_OPLUS_FEATURE_FTMMODE)
		pr_err("oplus_ftm_mode from cmdline : %s\n", oplus_ftm_mode);
		if (strcmp(oplus_ftm_mode, "factory2") == 0) {
			ftm_mode = MSM_BOOT_MODE__FACTORY;
			pr_err("kernel ftm OK\r\n");
		} else if (strcmp(oplus_ftm_mode, "ftmwifi") == 0) {
			ftm_mode = MSM_BOOT_MODE__WLAN;
		} else if (strcmp(oplus_ftm_mode, "ftmmos") == 0) {
			ftm_mode = MSM_BOOT_MODE__MOS;
		} else if (strcmp(oplus_ftm_mode, "ftmrf") == 0) {
			ftm_mode = MSM_BOOT_MODE__RF;
		} else if (strcmp(oplus_ftm_mode, "ftmrecovery") == 0) {
			ftm_mode = MSM_BOOT_MODE__RECOVERY;
		} else if (strcmp(oplus_ftm_mode, "ftmsilence") == 0) {
			ftm_mode = MSM_BOOT_MODE__SILENCE;
		} else if (strcmp(oplus_ftm_mode, "ftmsau") == 0) {
			ftm_mode = MSM_BOOT_MODE__SAU;
		} else if (strcmp(oplus_ftm_mode, "ftmaging") == 0) {
			ftm_mode = MSM_BOOT_MODE__AGING;
		} else if (strcmp(oplus_ftm_mode, "ftmsafe") == 0) {
			ftm_mode = MSM_BOOT_MODE__SAFE;
		}

		if ((ftm_mode == MSM_BOOT_MODE__FACTORY) \
			|| (ftm_mode == MSM_BOOT_MODE__WLAN) \
			|| (ftm_mode == MSM_BOOT_MODE__RF)) {
			pr_err("oplus_ftm_mode set console level to silent\n");
			console_printk[0] = CONSOLE_LOGLEVEL_SILENT;
		}
#endif
#else
    char *substr;

	substr = strstr(boot_command_line, "oplus_ftm_mode=");
	if (substr) {
		substr += strlen("oplus_ftm_mode=");

		if (strncmp(substr, "factory2", 5) == 0) {
			ftm_mode = MSM_BOOT_MODE__FACTORY;
			pr_err("kernel ftm OK\r\n");
		} else if (strncmp(substr, "ftmwifi", 5) == 0) {
			ftm_mode = MSM_BOOT_MODE__WLAN;
		} else if (strncmp(substr, "ftmmos", 5) == 0) {
			ftm_mode = MSM_BOOT_MODE__MOS;
		} else if (strncmp(substr, "ftmrf", 5) == 0) {
			ftm_mode = MSM_BOOT_MODE__RF;
		} else if (strncmp(substr, "ftmrecovery", 5) == 0) {
			ftm_mode = MSM_BOOT_MODE__RECOVERY;
		} else if (strncmp(substr, "ftmsilence", 10) == 0) {
			ftm_mode = MSM_BOOT_MODE__SILENCE;
		} else if (strncmp(substr, "ftmsau", 6) == 0) {
			ftm_mode = MSM_BOOT_MODE__SAU;
        //xiaofan.yang@PSW.TECH.AgingTest, 2019/01/07,Add for factory agingtest
        } else if (strncmp(substr, "ftmaging", 8) == 0) {
            ftm_mode = MSM_BOOT_MODE__AGING;
		} else if (strncmp(substr, "ftmsafe", 7) == 0) {
			ftm_mode = MSM_BOOT_MODE__SAFE;
		}
	}
#endif
	pr_err("board_ftm_mode_init ftm_mode=%d\n", ftm_mode);
	return 0;
}

int get_boot_mode(void)
{
	return ftm_mode;
}

EXPORT_SYMBOL(get_boot_mode);
static ssize_t ftmmode_show(struct kobject *kobj, struct kobj_attribute *attr,
								 char *buf)
{
	return sprintf(buf, "%d\n", ftm_mode);
}

struct kobj_attribute ftmmode_attr = {
	.attr = {"ftmmode", 0444},
	.show = &ftmmode_show,
};

static struct attribute * g[] = {
	&ftmmode_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

char pwron_event[MAX_CMD_LENGTH + 1];
static int __init start_reason_init(void)
{
#if IS_MODULE(CONFIG_OPLUS_FEATURE_PROJECTINFO)	
#if IS_MODULE(CONFIG_OPLUS_FEATURE_OPLUSBOOT)
	pr_err("startup_mode from cmdline : %s\n", startup_mode);
	strcpy(pwron_event, startup_mode);
	pwron_event[strlen(startup_mode)] = '\0';
	pr_info("parse poweron reason %s i = %llu\n", pwron_event, strlen(startup_mode));
#endif
#else
    int i;
	char * substr = strstr(boot_command_line, "oplusboot.startupmode=");
	if (NULL == substr) {
		return 0;
	}
	substr += strlen("oplusboot.startupmode=");
	for (i=0; substr[i] != ' ' && i < MAX_CMD_LENGTH && substr[i] != '\0'; i++) {
		pwron_event[i] = substr[i];
	}

	pwron_event[i] = '\0';
	pr_info("parse poweron reason %s i = %d\n", pwron_event, i);
#endif

	return 0;
}

char boot_mode[MAX_CMD_LENGTH + 1];
bool qpnp_is_power_off_charging(void)
{
	if (!strcmp(boot_mode, "charger")) {
		return true;
	}

	return false;
}
EXPORT_SYMBOL(qpnp_is_power_off_charging);

bool op_is_monitorable_boot(void)
{
	if (ftm_mode != MSM_BOOT_MODE__NORMAL) {
		return false;
	}

	if (!strcmp(boot_mode, "normal")) {
		return true;
	} else if (!strcmp(boot_mode, "reboot")) {
		return true;
	} else if (!strcmp(boot_mode, "kernel")) {
		return true;
	} else if (!strcmp(boot_mode, "rtc")) {
		return true;
	} else {
		return false;
	}
}
EXPORT_SYMBOL(op_is_monitorable_boot);

char charger_reboot[MAX_CMD_LENGTH + 1];
bool qpnp_is_charger_reboot(void)
{
	pr_err("%s charger_reboot:%s\n", __func__, charger_reboot);
	if (!strcmp(charger_reboot, "1")) {
		return true;
	}

	return false;
}
EXPORT_SYMBOL(qpnp_is_charger_reboot);

static int __init oplus_charger_reboot(void)
{
#if IS_MODULE(CONFIG_OPLUS_FEATURE_PROJECTINFO)
#ifdef CONFIG_OPLUS_FEATURE_CHARGERPRESENT
	pr_err("charger present from cmdline : %s\n", charger_present);
	strcpy(charger_reboot, charger_present);
	charger_reboot[strlen(charger_present)] = '\0';
	pr_info("%s: parse charger_reboot %s\n", __func__, charger_reboot);
#endif
#else
    int i;
	char * substr = strstr(boot_command_line, "oplus_charger_present=");
	if (substr) {
		substr += strlen("oplus_charger_present=");
		for (i=0; substr[i] != ' '&& i < MAX_CMD_LENGTH && substr[i] != '\0'; i++) {
			charger_reboot[i] = substr[i];
		}
		charger_reboot[i] = '\0';
		pr_info("%s: parse charger_reboot %s\n", __func__, charger_reboot);
	}
#endif

	return 0;
}

int __init  board_boot_mode_init(void)
{
#if IS_MODULE(CONFIG_OPLUS_FEATURE_PROJECTINFO)
#if IS_MODULE(CONFIG_OPLUS_FEATURE_OPLUSBOOT)
	pr_err("mode from cmdline : %s\n", bootmode);
	strcpy(boot_mode, bootmode);
	boot_mode[strlen(bootmode)] = '\0';
	pr_err("oplusboot.mode= %s\n", boot_mode);
#endif
#else
    int i;
	char *substr;

	substr = strstr(boot_command_line, "oplusboot.mode=");
	if (substr) {
		substr += strlen("oplusboot.mode=");
		for (i=0; substr[i] != ' ' && i < MAX_CMD_LENGTH && substr[i] != '\0'; i++) {
			boot_mode[i] = substr[i];
		}
		boot_mode[i] = '\0';
		pr_err("oplusboot.mode= %s\n", boot_mode);
	}
#endif

	return 0;
}

static int __init boot_mode_init(void)
{
	int rc = 0;

	pr_info("%s: parse boot_mode\n", __func__);

	board_boot_mode_init();
	board_ftm_mode_init();
	start_reason_init();
	oplus_charger_reboot();

	systeminfo_kobj = kobject_create_and_add("systeminfo", NULL);
	if (systeminfo_kobj) {
		rc = sysfs_create_group(systeminfo_kobj, &attr_group);
	}
	return rc;
}

arch_initcall(boot_mode_init);

#else

static int __init boot_mode_init(void)
{
	return 0;
}
arch_initcall(boot_mode_init);
#endif /* CONFIG_OPLUS_SYSTEM_KERNEL_QCOM */

MODULE_LICENSE("GPL v2");
