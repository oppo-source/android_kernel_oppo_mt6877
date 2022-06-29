// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/capability.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/pm_wakeirq.h>
#include <linux/types.h>
#include <trace/events/power.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/delay.h>
#include <linux/mtk_dbg_common_v1.h>
#include <linux/mtk_ccci_common.h>
#ifdef CONFIG_POWER_6877_VERSION
#include "oplus_wakelock_profiler_garen.h"
#else
#include "oplus_wakelock_profiler_mtk.h"
#endif
#define PR_INFO(fmt, ...) \
	pr_info("[WAKELOCK_PROFILE]%s: " fmt, __func__, ##__VA_ARGS__)

#define PRINT_ALL_STATIC_INFO  ~(0x00000000)
#define print_all_detail_info  true
#define print_module_info  false
#define PRINT_THREAD_CONTROL 180
struct timespec64 ts_start;
struct timespec64 ts_end;
struct timespec64 statisticstime_start;
static int64_t screen_off_time;
static int64_t real_statistics_time;

/*---------------------Function: When ap suspend,statstics wakeup reason---------------------*/

void pmic_irq_count_function_init()
{
	int i = 0;

	for (i = 0; i < pmic_irq_info.ws_number; i++) {
		pmic_irq_info.pmic_irq_desc[i].name = pmic_irq_name[i];
	}

	return;
}

int wakeup_reasons_statics(const char *irq_name, u64 choose_flag)
{
	int i, j, k = 0;
	int state = false;
	struct wakeup_count_desc_t *desc;
	struct ws_desc_t *ws_desc;

	if (irq_name == NULL) {
		return state;
	}

	PR_INFO("irq_name=%s, choose_flag=0x%x", irq_name, choose_flag);
	/*statics pmic_irq*/
	if (choose_flag & pmic_irq_info.module_mask) {
		pmic_irq_info.module_all_count++;
		for (k = 0; k < pmic_irq_info.ws_number; k++) {
			if (!strncmp(irq_name, pmic_irq_info.pmic_irq_desc[k].name,
					strlen(pmic_irq_info.pmic_irq_desc[k].name)) &&
				strncmp(pmic_irq_info.pmic_irq_desc[k].name, "MT6358_IRQ_PWRKEY",
					sizeof("MT6358_IRQ_PWRKEY")) &&
				strncmp(pmic_irq_info.pmic_irq_desc[k].name, "MT6358_IRQ_PWRKEY_R",
					sizeof("MT6358_IRQ_PWRKEY_R"))
				&&
				strncmp(pmic_irq_info.pmic_irq_desc[k].name, "MT6358_IRQ_RTC",
					sizeof("MT6358_IRQ_RTC"))) {
				pmic_irq_info.pmic_irq_desc[k].count++;
				pmic_irq_info.pmic_irq_desc[k].no_clear_resume_count++;
				wc_sum.module_all_count++;
				state = true;
			}
		}
	}

	for (i = 0; (desc = all_modules[i]) != NULL; i++) {
		if (desc->module_mask & choose_flag) {
			for (j = 0; j < desc->ws_number; j++) {
				ws_desc = &desc->ws_desc[j];

				if (!strncmp(irq_name, ws_desc->name, strlen(ws_desc->name))) {
					ws_desc->count++;
					desc->module_all_count++;
					desc->no_clear_resume_count++;
					wc_sum.module_all_count++;
					state = true;
				}
			}
		}
	}

	return state;
}

void wakeup_reasons_clear(u64 choose_flag)
{
	int i, j, k = 0;
	struct wakeup_count_desc_t *desc;
	struct ws_desc_t *ws_desc;

	PR_INFO("choose_flag=0x%x", choose_flag);

	/*clear pmic_irq count*/
	if (pmic_irq_info.module_mask & choose_flag) {
		pmic_irq_info.module_all_count = 0;

		for (k = 0; k < pmic_irq_info.ws_number; k++) {
			pmic_irq_info.pmic_irq_desc[k].count = 0;
		}
	}

	/*clear pmic_irq count*/
	for (i = 0; (desc = all_modules[i]) != NULL; i++) {
		if (desc->module_mask & choose_flag) {
			for (j = 0; j < desc->ws_number; j++) {
				ws_desc = &desc->ws_desc[j];
				ws_desc->count = 0;
			}

			desc->module_all_count = 0;
		}
	}

	awuc.alarm_count = 0;
	awuc.alarm_wakeup_count = 0;
}

void wakeup_reasons_print(u64 choose_flag, bool detail)
{
	int i, j, k = 0;
	struct wakeup_count_desc_t *desc;
	PR_INFO("...\n");

	/*print pmic_irq count*/
	if ((pmic_irq_info.module_mask & choose_flag)
		&& pmic_irq_info.module_all_count != 0) {
		for (k = 0; k < pmic_irq_info.ws_number; k++) {
			if (pmic_irq_info.pmic_irq_desc[k].count != 0) {
				PR_INFO("---->%s %lld times,\n", pmic_irq_info.pmic_irq_desc[k].name,
					pmic_irq_info.pmic_irq_desc[k].count);
			}
		}
	}

	/*print pmic_irq count*/
	for (i = 0; (desc = all_modules[i]) != NULL; i++) {
		if (desc->module_mask & choose_flag) {
			PR_INFO("---->%s %lld times,\n", desc->module_name, desc->module_all_count);

			if (detail) {
				PR_INFO("----->detail:\n");

				for (j = 0; j < desc->ws_number; j++) {
					if (desc->ws_desc[j].prop & (IRQ_PROP_REAL | IRQ_PROP_EXCHANGE)) {
						PR_INFO("------>%s wakeup %lld times\n", desc->ws_desc[j].name,
							desc->ws_desc[j].count);
					}
				}
			}
		}
	}
}

static ssize_t ap_resume_reason_stastics_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	int i, j, buf_offset = 0;
	struct wakeup_count_desc_t *desc;

	/*add for pmic irq static*/
	if (pmic_irq_info.module_all_count != 0 && buf_offset < PAGE_SIZE) {
		for (j = 0; j < pmic_irq_info.ws_number; j++) {
			if (pmic_irq_info.pmic_irq_desc[j].count != 0) {
				buf_offset += sprintf(buf + buf_offset, "%s: %lld\n",
						pmic_irq_info.pmic_irq_desc[j].name,
						pmic_irq_info.pmic_irq_desc[j].count);
			}
		}
	}

	/*add for pmic irq static*/
	for (i = 0; (desc = all_modules[i]) != NULL && buf_offset < PAGE_SIZE; i++) {
		if (!strncmp(desc->module_name, "abort", strlen("abort"))
			|| !strncmp(desc->module_name, "other", strlen("other"))
			|| !strncmp(desc->module_name, "wakeup_sum", strlen("wakeup_sum"))) {
			buf_offset += sprintf(buf + buf_offset, "%s= %lld\n", desc->module_name,
					desc->module_all_count);
			PR_INFO("%s= %lld times.\n", desc->module_name, desc->module_all_count);

		} else {
			buf_offset += sprintf(buf + buf_offset, "%s: %lld\n", desc->module_name,
					desc->module_all_count);
			PR_INFO("%s: %lld times.\n", desc->module_name, desc->module_all_count);
		}
	}

	return buf_offset;
}

/*echo reset >   /sys/kernel/wakeup_reasons/wakeup_stastisc_reset*/
static ssize_t wakeup_stastisc_reset_store(
	struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
	size_t count)
{
	const char *pstring = "reset";

	if (!(count == strlen(pstring) || (count == strlen(pstring) + 1
				&& buf[count - 1] == '\n'))) {
		return count;
	}

	if (strncmp(buf, pstring, strlen(pstring))) {
		return count;
	}

	wakeup_reasons_clear(WS_CNT_ALL);

	return count;
}

static struct kobj_attribute ap_resume_reason_stastics = __ATTR_RO(
		ap_resume_reason_stastics);
static struct kobj_attribute wakeup_stastisc_reset = __ATTR(
		wakeup_stastisc_reset,
		S_IWUSR | S_IRUGO, NULL, wakeup_stastisc_reset_store);

/*---------------------Function: When ap suspend,statstics wakeup reason---------------------*/



/*----------------------Function: statstics alarmtimer wakeup stastics ----------------------*/

static int alarmtimer_suspend_flag_get(void)
{
	return atomic_read(&awuc.suspend_flag);
}
void alarmtimer_suspend_flag_set(void)
{
	atomic_set(&awuc.suspend_flag, 1);
}
void alarmtimer_suspend_flag_clear(void)
{
	atomic_set(&awuc.suspend_flag, 0);
}
static int alarmtimer_busy_flag_get(void)
{
	return atomic_read(&awuc.busy_flag);
}
void alarmtimer_busy_flag_set(void)
{
	atomic_set(&awuc.busy_flag, 1);
}
void alarmtimer_busy_flag_clear(void)
{
	atomic_set(&awuc.busy_flag, 0);
}

void alarmtimer_wakeup_count(struct alarm *alarm)
{
	if (alarm->type == ALARM_REALTIME || alarm->type == ALARM_BOOTTIME) {
		awuc.alarm_count++;

		if (alarmtimer_suspend_flag_get() || alarmtimer_busy_flag_get()) {
			awuc.alarm_wakeup_count++;
			wakeup_reasons_statics(IRQ_NAME_RTCALARM, WS_CNT_RTCALARM);

			if (alarmtimer_busy_flag_get()) {
				alarmtimer_busy_flag_clear();
			}

			if (alarm->function) {
				PR_INFO("alarm_type=%d, not_netalarm_count=%lld, not_netalarm_wakeup_count=%lld,owner=%s,alarm_func=%pf\n",
					alarm->type, awuc.alarm_count, awuc.alarm_wakeup_count, alarm->comm,
					alarm->function);
			}
		}
	}
}
/*----------------------Function: statstics alarmtimer wakeup stastics ----------------------*/



/*--------------Function: When 26M CLK suspend ratio is 0,statstics which clk block--------------*/
void clk_state_statics(const u32 clk_state, const u64 ap_sleep_time,
	const u64 deep_sleep_time,
	const char *static_type)
{
	int i, j, len = 0;
	struct clk_state_desc_t *module_desc;
	struct clk_desc_t *clk_desc;

	PR_INFO("clk_state=0x%x, static_type=%s, ap_sleep_time=%lld, 26m_sleep_time=%lld.\n",
		clk_state,
		static_type, ap_sleep_time, deep_sleep_time);

	if ((strlen(static_type) == 0) || ap_sleep_time < 0 || deep_sleep_time < 0) {
		return;
	}

	len = sizeof(all_clk_info) / sizeof(all_clk_info[0]);

	if ((!strncmp(static_type, "static_clk_sleep_count",
				strlen("static_clk_sleep_count")))
		&& (clk_state & 0x2)) {
		normal_SleepClk_count_static.module_clk_count++;
		normal_SleepClk_count_static.module_ap_sleep_time += ap_sleep_time;
		normal_SleepClk_count_static.module_deep_sleep_time += deep_sleep_time;
		normal_SleepClk_count_static.clk_desc[0].clk_count++;
		normal_SleepClk_count_static.clk_desc[0].clk_time += ap_sleep_time;
		return;

	} else if (!strncmp(static_type, "static_clk_no_sleep",
			strlen("static_clk_no_sleep"))) {
		sum_clk_no_sleep.module_clk_count++;
		sum_clk_no_sleep.module_ap_sleep_time += ap_sleep_time;
		sum_clk_no_sleep.module_deep_sleep_time += deep_sleep_time;
		sum_clk_no_sleep.clk_desc[0].clk_count++;
		sum_clk_no_sleep.clk_desc[0].clk_time += ap_sleep_time;

		for (i = 0; i < len - 2;i++) { /*don't static sum_clk_no_sleep and normal_SleepClk_count_static*/
			module_desc = all_clk_info[i];

			if ((module_desc->module_clk_mask & clk_state) &&
				strncmp(module_desc->module_clk_name, "other_clk", strlen("other_clk")) &&
				strncmp(module_desc->module_clk_name, "static_clk_sleep_count",
					strlen("static_clk_sleep_count"))) {
				module_desc->module_clk_count++;
				module_desc->module_ap_sleep_time += ap_sleep_time;

				for (j = 0; j < module_desc->module_clk_num; j++) {
					clk_desc = &module_desc->clk_desc[j];

					if ((clk_desc->clk_mask)&clk_state) {
						clk_desc->clk_count++;
						clk_desc->clk_time += ap_sleep_time;
					}
				}

			} else if (!strncmp(module_desc->module_clk_name, "other_clk",
					strlen("other_clk"))
				&& (clk_state == OTHER_CLK)) {
				module_desc->module_clk_count++;
				module_desc->module_ap_sleep_time += ap_sleep_time;

				for (j = 0; j < module_desc->module_clk_num; j++) {
					clk_desc = &module_desc->clk_desc[j];

					if ((clk_desc->clk_mask)&clk_state) {
						clk_desc->clk_count++;
						clk_desc->clk_time += ap_sleep_time;
					}
				}
			}
		}

		return;
	}

	PR_INFO("error use");
}


void clk_sleep_state_clear(int choose_flag)
{
	int i, j, len;
	struct clk_state_desc_t *desc;
	struct clk_desc_t *clk_desc;

	PR_INFO("choose_flag=0x%x", choose_flag);
	len = sizeof(all_clk_info) / sizeof(all_clk_info[0]);

	for (i = 0; i < len; i++) {
		desc = all_clk_info[i];

		if (desc->module_clk_mask & choose_flag) {
			for (j = 0; j < desc->module_clk_num; j++) {
				clk_desc = &desc->clk_desc[j];
				clk_desc->clk_count = 0;
				clk_desc->clk_time = 0;
			}

			desc->module_clk_count = 0;
			desc->module_ap_sleep_time = 0;
			desc->module_deep_sleep_time = 0;
		}
	}
}

static void dump_clk_state(int choose_flag, bool detail)
{
	int i, j, len;
	struct clk_state_desc_t *desc;
	PR_INFO("...\n");
	len = sizeof(all_clk_info) / sizeof(all_clk_info[0]);

	for (i = 0; i < len; i++) {
		desc = all_clk_info[i];

		if (desc->module_clk_mask & choose_flag) {
			if (!strncmp(desc->module_clk_name, "sum_clk_no_sleep_count",
					strlen("sum_clk_no_sleep_count"))) {
				PR_INFO("---->%s %lld times : %lld(s)\n", desc->module_clk_name,
					desc->module_clk_count,
					desc->module_ap_sleep_time / SUSPEND_RTC_CLK);

			} else if (!strncmp(desc->module_clk_name, "sum_all_clk_sleep_count",
					strlen("sum_all_clk_sleep_count"))) {
				PR_INFO("---->%s %lld times : %lld(s)\n", desc->module_clk_name,
					desc->module_clk_count,
					desc->module_ap_sleep_time / SUSPEND_RTC_CLK);
				PR_INFO("---->ap_sleep_time= %lld(s)\n",
					(normal_SleepClk_count_static.module_ap_sleep_time +
						sum_clk_no_sleep.module_ap_sleep_time) /
					SUSPEND_RTC_CLK);
				PR_INFO("---->deep_sleep_time= %lld(s)\n",
					(normal_SleepClk_count_static.module_deep_sleep_time +
						sum_clk_no_sleep.module_deep_sleep_time) /
					SUSPEND_RTC_CLK);

			} else {
				PR_INFO("---->%s %lld times : %lld(s)\n", desc->module_clk_name,
					desc->module_clk_count,
					desc->module_ap_sleep_time / SUSPEND_RTC_CLK);

				if (detail) {
					PR_INFO("----->%s detail info:\n", desc->module_clk_name);

					for (j = 0; j < desc->module_clk_num; j++) {
						if (desc->clk_desc[j].clk_mask & choose_flag) {
							PR_INFO("------>%s no_sleep %lld times,block %lld (s)\n",
								desc->clk_desc[j].clk_name,
								desc->clk_desc[j].clk_count, desc->clk_desc[j].clk_time / SUSPEND_RTC_CLK);
						}
					}
				}
			}
		}
	}
}

static ssize_t clk_sleep_state_stastics_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	int i, len, buf_offset = 0;
	struct clk_state_desc_t *desc;
	len = sizeof(all_clk_info) / sizeof(all_clk_info[0]);

	for (i = 0; (i < len && buf_offset < PAGE_SIZE); i++) {
		desc = all_clk_info[i];

		if (!strncmp(desc->module_clk_name, "sum_clk_no_sleep_count",
				strlen("sum_clk_no_sleep_count"))) {
			buf_offset += sprintf(buf + buf_offset, "%s= %lld : %lld\n",
					desc->module_clk_name,
					desc->module_clk_count, desc->module_ap_sleep_time / SUSPEND_RTC_CLK);
			PR_INFO("%s: %lld times : %lld(s).\n", desc->module_clk_name,
				desc->module_clk_count,
				desc->module_ap_sleep_time / SUSPEND_RTC_CLK);

		} else if (!strncmp(desc->module_clk_name, "sum_all_clk_sleep_count",
				strlen("sum_all_clk_sleep_count"))) {
			buf_offset += sprintf(buf + buf_offset, "%s= %lld : %lld\n",
					desc->module_clk_name,
					desc->module_clk_count, desc->module_ap_sleep_time / SUSPEND_RTC_CLK);
			buf_offset += sprintf(buf + buf_offset, "ap_sleep_time= %lld\n",
					(normal_SleepClk_count_static.module_ap_sleep_time +
						sum_clk_no_sleep.module_ap_sleep_time) /
					SUSPEND_RTC_CLK);
			buf_offset += sprintf(buf + buf_offset, "deep_sleep_time= %lld\n",
					(normal_SleepClk_count_static.module_deep_sleep_time +
						sum_clk_no_sleep.module_deep_sleep_time) /
					SUSPEND_RTC_CLK);
			PR_INFO("%s: %lld times : %lld(s).\n", desc->module_clk_name,
				desc->module_clk_count,
				desc->module_ap_sleep_time / SUSPEND_RTC_CLK);
			PR_INFO("ap_sleep_time= %lld(s).\n",
				(normal_SleepClk_count_static.module_ap_sleep_time +
					sum_clk_no_sleep.module_ap_sleep_time) /
				SUSPEND_RTC_CLK);
			PR_INFO("deep_sleep_time= %lld(s).\n",
				(normal_SleepClk_count_static.module_deep_sleep_time +
					sum_clk_no_sleep.module_deep_sleep_time) /
				SUSPEND_RTC_CLK);

		} else {
			buf_offset += sprintf(buf + buf_offset, "%s: %lld : %lld\n",
					desc->module_clk_name,
					desc->module_clk_count, desc->module_ap_sleep_time / SUSPEND_RTC_CLK);
			PR_INFO("%s: %lld times : %lld(s).\n", desc->module_clk_name,
				desc->module_clk_count,
				desc->module_ap_sleep_time / SUSPEND_RTC_CLK);
		}
	}

	return buf_offset;
}


/*echo reset >   /sys/kernel/wakeup_reasons/clkstate_stastics_reset*/
static ssize_t clkstate_stastics_reset_store(
	struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
	size_t count)
{
	const char *pstring = "reset";

	if (!(count == strlen(pstring) || (count == strlen(pstring) + 1
				&& buf[count - 1] == '\n'))) {
		return count;
	}

	if (strncmp(buf, pstring, strlen(pstring))) {
		return count;
	}

	clk_sleep_state_clear(CLK_CNT_CLEAR);
	return count;
}


static struct kobj_attribute clk_sleep_state_stastics = __ATTR_RO(
		clk_sleep_state_stastics);
static struct kobj_attribute clkstate_stastics_reset = __ATTR(
		clkstate_stastics_reset,
		S_IWUSR | S_IRUGO, NULL, clkstate_stastics_reset_store);

/*--------------Function: When 26M CLK suspend ratio is 0,statstics which clk block--------------*/



/*-------------------Function: statstics vmin/ap/modem suspend ratio info-------------------*/

void oplus_rpmh_stats_statics(const char *rpm_name, u64 sleep_count,
	u64 sleep_time)
{
	int i, len = 0;
	struct rpmh_state_desc_t *desc;

	if (rpm_name == NULL) {
		return;
	}

	PR_INFO("rpm_name=%s,sleep_count=%lld,sleep_time=%lld", rpm_name, sleep_count,
		sleep_time);
	len = sizeof(all_rpmh_statics_info) / sizeof(all_rpmh_statics_info[0]);

	for (i = 0; i < len; i++) {
		desc = all_rpmh_statics_info[i];

		if (!strncmp(desc->module_name, rpm_name, strlen(rpm_name))) {
			desc->module_sleep_count += sleep_count;
			desc->module_sleep_time += sleep_time;
		}
	}

	return;
}

EXPORT_SYMBOL(oplus_rpmh_stats_statics);

static void dump_rpmh_state()
{
	int i, len, suspendrate;
	struct rpmh_state_desc_t *desc;
	PR_INFO("...\n");
	len = sizeof(all_rpmh_statics_info) / sizeof(all_rpmh_statics_info[0]);

	for (i = 0; i < len; i++) {
		desc = all_rpmh_statics_info[i];

		if (!strncmp(desc->module_name, "Apss", strlen("Apss"))) {
			desc->module_sleep_time = (normal_SleepClk_count_static.module_ap_sleep_time +
					sum_clk_no_sleep.module_ap_sleep_time);

		} else if (!strncmp(desc->module_name, "vmin", strlen("vmin"))) {
			desc->module_sleep_time = (normal_SleepClk_count_static.module_deep_sleep_time +
					sum_clk_no_sleep.module_deep_sleep_time);
		}

		if (real_statistics_time == 0) {
			suspendrate = 0;

		} else {
			suspendrate = (real_statistics_time - (desc->module_sleep_time /
						SUSPEND_RTC_CLK)) >= 0 ? ((
						desc->module_sleep_time / SUSPEND_RTC_CLK) * 100 / real_statistics_time) : 100;
		}

		PR_INFO("--->%s : %lld(times) : %lld(s): %d(rate)\n", desc->module_name,
			desc->module_sleep_count,
			desc->module_sleep_time / SUSPEND_RTC_CLK, suspendrate);
	}

	PR_INFO("--->real_statistics_time=%lld(s)\n", real_statistics_time);
	PR_INFO("--->Screen_off_time=%lld(s)\n", screen_off_time);
}

void oplus_rpm_stats_statics_clear()
{
	int i, len;
	struct rpmh_state_desc_t *desc;

	PR_INFO("...\n");
	len = sizeof(all_rpmh_statics_info) / sizeof(all_rpmh_statics_info[0]);

	for (i = 0; i < len; i++) {
		desc = all_rpmh_statics_info[i];
		desc->module_sleep_count = 0;
		desc->module_sleep_time = 0;
	}

	screen_off_time = 0;
	real_statistics_time = 0;
	return;
}

static ssize_t oplus_rpm_stats_statics_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	int i, len, buf_offset = 0;
	struct rpmh_state_desc_t *desc;
	len = sizeof(all_rpmh_statics_info) / sizeof(all_rpmh_statics_info[0]);

	for (i = 0; (i < len && buf_offset < PAGE_SIZE); i++) {
		desc = all_rpmh_statics_info[i];
		buf_offset += sprintf(buf + buf_offset, "%s: %lld : %lld\n", desc->module_name,
				desc->module_sleep_count, desc->module_sleep_time / SUSPEND_RTC_CLK);
		PR_INFO("%s: %lld times : %lld(s).\n", desc->module_name,
			desc->module_sleep_count,
			desc->module_sleep_time / SUSPEND_RTC_CLK);
	}

	PR_INFO("real_statistics_time=%lld(s)\n", real_statistics_time);
	buf_offset += sprintf(buf + buf_offset, "real_statistics_time=%lld(s)\n",
			real_statistics_time);
	PR_INFO("Screen_off_time=%lld(s)\n", screen_off_time);
	buf_offset += sprintf(buf + buf_offset, "Screen_off_time=%lld(s)\n",
			screen_off_time);
	return buf_offset;
}
/*echo reset >   /sys/kernel/wakeup_reasons/clkstate_stastics_reset*/
static ssize_t oplus_rpm_stats_statics_reset_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	const char *pstring = "reset";

	if (!(count == strlen(pstring) || (count == strlen(pstring) + 1
				&& buf[count - 1] == '\n'))) {
		return count;
	}

	if (strncmp(buf, pstring, strlen(pstring))) {
		return count;
	}

	oplus_rpm_stats_statics_clear();
	return count;
}


static struct kobj_attribute oplus_rpm_stats_statics = __ATTR_RO(
		oplus_rpm_stats_statics);
static struct kobj_attribute oplus_rpm_stats_statics_reset = __ATTR(
		oplus_rpm_stats_statics_reset,
		S_IWUSR | S_IRUGO, NULL, oplus_rpm_stats_statics_reset_store);

/*-------------------Function: statstics vmin/ap/modem suspend ratio info-------------------*/



/* ---------------------Function: add for midas wakeup reasons stastisc ---------------------*/

void all_wakeup_reasons_clear(u64 choose_flag)
{
	int i, j, k = 0;
	struct wakeup_count_desc_t *desc;
	struct ws_desc_t *ws_desc;

	PR_INFO("choose_flag=0x%x", choose_flag);

	/*clear pmic_irq count*/
	if (pmic_irq_info.module_mask & choose_flag) {
		for (k = 0; k < pmic_irq_info.ws_number; k++) {
			pmic_irq_info.pmic_irq_desc[k].no_clear_resume_count = 0;
		}
	}

	/*clear pmic_irq count*/
	for (i = 0; (desc = all_modules[i]) != NULL; i++) {
		if (desc->module_mask & choose_flag) {
			desc->no_clear_resume_count = 0;
		}
	}
}


static ssize_t all_ap_resume_reason_stastics_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	int i, j, buf_offset = 0;
	struct wakeup_count_desc_t *desc;

	/*add for pmic irq static*/
	if (pmic_irq_info.module_all_count != 0 && buf_offset < PAGE_SIZE) {
		for (j = 0; j < pmic_irq_info.ws_number; j++) {
			if (pmic_irq_info.pmic_irq_desc[j].count != 0) {
				buf_offset += sprintf(buf + buf_offset, "%s: %lld\n",
						pmic_irq_info.pmic_irq_desc[j].name,
						pmic_irq_info.pmic_irq_desc[j].no_clear_resume_count);
				/*PR_INFO("MIDAS stastics:%s= %lld times.\n", pmic_irq_info.pmic_irq_desc[j].name, pmic_irq_info.pmic_irq_desc[j].no_clear_resume_count);*/
			}
		}
	}
	/*add for pmic irq static*/
	for (i = 0; (desc = all_modules[i]) != NULL && buf_offset < PAGE_SIZE; i++) {
		if (!strncmp(desc->module_name, "abort", strlen("abort"))
			|| !strncmp(desc->module_name, "other", strlen("other"))) {
			buf_offset += sprintf(buf + buf_offset, "%s= %lld\n", desc->module_name,
					desc->no_clear_resume_count);
			/*PR_INFO("MIDAS stastics:%s= %lld times.\n", desc->module_name, desc->no_clear_resume_count);*/
		} else if (!strncmp(desc->module_name, "wakeup_sum", strlen("wakeup_sum"))) {
			continue;
		}
		else {
			buf_offset += sprintf(buf + buf_offset, "%s: %lld\n", desc->module_name,
					desc->no_clear_resume_count);
			/*PR_INFO("MIDAS stastics:%s: %lld times.\n", desc->module_name, desc->no_clear_resume_count);*/
		}
	}

	return buf_offset;
}


static ssize_t all_resume_reason_stastics_reset_store(
	struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
	size_t count)
{
	const char *pstring = "reset";

	if (!(count == strlen(pstring) || (count == strlen(pstring) + 1
				&& buf[count - 1] == '\n'))) {
		return count;
	}

	if (strncmp(buf, pstring, strlen(pstring))) {
		return count;
	}

	all_wakeup_reasons_clear(WS_CNT_ALL);

	return count;
}

static ssize_t all_resume_reason_stastics_reset_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf) {
	const char *pstring = "reset";
	return sprintf(buf, "%s\n", pstring);
}

static struct kobj_attribute all_ap_resume_reason_stastics = __ATTR_RO(all_ap_resume_reason_stastics);
static struct kobj_attribute all_resume_reason_stastics_reset = __ATTR_RW(all_resume_reason_stastics_reset);

/* ---------------------Function: add for midas wakeup reasons stastisc ---------------------*/



/*-------------------------Function: add for statistics kernel_time  -------------------------*/

extern inline bool ws_all_release(void);
struct ws_time_statistics {
	ktime_t start;
	ktime_t end;
	ktime_t hold;
	ktime_t reset;

	bool all_ws_released;
};
static DEFINE_SPINLOCK(ws_lock);
static struct ws_time_statistics wsts;

void wakeup_get_start_time(void)
{
	unsigned long flags;

	spin_lock_irqsave(&ws_lock, flags);

	if (wsts.all_ws_released) {
		wsts.all_ws_released = false;
		wsts.start = ktime_get();
	}

	spin_unlock_irqrestore(&ws_lock, flags);
}

void wakeup_get_end_hold_time(void)
{
	unsigned long flags;
	ktime_t ws_hold;

	spin_lock_irqsave(&ws_lock, flags);
	wsts.all_ws_released = true;
	wsts.end = ktime_get();
	ws_hold = ktime_sub(wsts.end, wsts.start);
	wsts.hold = ktime_add(wsts.hold, ws_hold);
	spin_unlock_irqrestore(&ws_lock, flags);
}
static void kernel_time_reset(void)
{
	unsigned long flags = 0;

	printk("%s\n", __func__);

	if (!ws_all_release()) {
		ktime_t offset_hold_time;
		ktime_t now = ktime_get();
		spin_lock_irqsave(&ws_lock, flags);
		offset_hold_time = ktime_sub(now, wsts.start);
		wsts.reset = ktime_add(wsts.hold, offset_hold_time);
		spin_unlock_irqrestore(&ws_lock, flags);

	} else {
		spin_lock_irqsave(&ws_lock, flags);
		wsts.reset = wsts.hold;
		spin_unlock_irqrestore(&ws_lock, flags);
	}
}

static ssize_t kernel_time_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	const char *pstring = "reset";

	if (!(count == strlen(pstring) || (count == strlen(pstring) + 1
				&& buf[count - 1] == '\n'))) {
		return count;
	}

	if (strncmp(buf, pstring, strlen(pstring))) {
		return count;
	}

	kernel_time_reset();
	return count;
}

static ssize_t kernel_time_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int buf_offset = 0;
	unsigned long flags = 0;
	ktime_t newest_hold_time;

	if (!ws_all_release()) {
		ktime_t offset_hold_time;
		ktime_t now = ktime_get();
		spin_lock_irqsave(&ws_lock, flags);

		offset_hold_time = ktime_sub(now, wsts.start);
		newest_hold_time = ktime_add(wsts.hold, offset_hold_time);

	} else {
		spin_lock_irqsave(&ws_lock, flags);
		newest_hold_time = wsts.hold;
	}

	newest_hold_time = ktime_sub(newest_hold_time, wsts.reset);
	spin_unlock_irqrestore(&ws_lock, flags);
	buf_offset += sprintf(buf + buf_offset, "%lld\n",
			ktime_to_ms(newest_hold_time));

	return buf_offset;
}
static struct kobj_attribute kernel_time = __ATTR_RW(kernel_time);

/*-------------------------Function: add for statistics kernel_time  -------------------------*/



/*-------------------Function: add for statistics TOP4 active wakeup source -------------------*/

#define STATICS_NUMBER 4
#define NAME_SIZE 40

extern void get_ws_listhead(struct list_head **ws);
extern void wakeup_srcu_read_lock(int *srcuidx);
extern void wakeup_srcu_read_unlock(int srcuidx);

struct wakelock_desc {
	struct wakeup_source *ws;
	ktime_t max_ws_time;
	char max_ws_name[NAME_SIZE];
};


static ktime_t active_max_reset_time;

static struct wakelock_desc max_wakelock_list[STATICS_NUMBER];
static DEFINE_MUTEX(max_wakelock_list_lock);


static void active_max_reset(void)
{
	int srcuidx;
	unsigned long flags;
	ktime_t total_time;
	struct wakeup_source *ws;
	struct list_head *ws_listhead;

	printk("%s\n", __func__);
	get_ws_listhead(&ws_listhead);
	mutex_lock(&max_wakelock_list_lock);
	active_max_reset_time = ktime_get();
	mutex_unlock(&max_wakelock_list_lock);
	wakeup_srcu_read_lock(&srcuidx);
	list_for_each_entry_rcu(ws, ws_listhead, entry) {
		spin_lock_irqsave(&ws->lock, flags);
		total_time = ws->total_time;

		if (ws->active) {
			ktime_t active_time;
			ktime_t now = ktime_get();
			active_time = ktime_sub(now, ws->last_time);
			total_time = ktime_add(ws->total_time, active_time);
		}

		ws->total_time_backup = total_time;
		spin_unlock_irqrestore(&ws->lock, flags);


	}
	wakeup_srcu_read_unlock(srcuidx);
}

static ssize_t active_max_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int srcuidx, i, j;
	int max_ws_rate;
	ktime_t cur_ws_total_time, cur_ws_total_time_backup;
	ktime_t wall_delta;
	int buf_offset = 0;
	unsigned long flags;
	struct wakeup_source *ws;
	struct list_head *ws_listhead;

	get_ws_listhead(&ws_listhead);
	mutex_lock(&max_wakelock_list_lock);
	memset(max_wakelock_list, 0, sizeof(max_wakelock_list));
	wakeup_srcu_read_lock(&srcuidx);
	list_for_each_entry_rcu(ws, ws_listhead, entry) {
		ktime_t active_time;
		ktime_t now = ktime_get();
		spin_lock_irqsave(&ws->lock, flags);
		cur_ws_total_time = ws->total_time;

		if (ws->active) {
			active_time = ktime_sub(now, ws->last_time);
			cur_ws_total_time = ktime_add(ws->total_time, active_time);
		}

		cur_ws_total_time_backup = ws->total_time_backup;
		spin_unlock_irqrestore(&ws->lock, flags);

		if (ktime_compare(cur_ws_total_time, cur_ws_total_time_backup) >= 0) {
			for (i = 0; i < STATICS_NUMBER; i++) {
				if (ktime_compare(ktime_sub(cur_ws_total_time, cur_ws_total_time_backup),
						max_wakelock_list[i].max_ws_time) > 0) {
					for (j = STATICS_NUMBER - 1 ; j >= i + 1; j--) {
						max_wakelock_list[j].ws = max_wakelock_list[j - 1].ws;
						max_wakelock_list[j].max_ws_time = max_wakelock_list[j - 1].max_ws_time;
					}
					max_wakelock_list[i].ws = ws;
					max_wakelock_list[i].max_ws_time = ktime_sub(cur_ws_total_time,
							cur_ws_total_time_backup);
					break;
				}
			}
		}
	}

	for (i = 0; i < STATICS_NUMBER; i++) {
		if ((max_wakelock_list[i].ws != NULL)
			&& (max_wakelock_list[i].ws->name != NULL)) {
			scnprintf(max_wakelock_list[i].max_ws_name, NAME_SIZE - 1, "%s",
				max_wakelock_list[i].ws->name);
		}
	}

	wakeup_srcu_read_unlock(srcuidx);

	wall_delta = ktime_sub(ktime_get(), active_max_reset_time);
	buf_offset += sprintf(buf + buf_offset, "Name\tTime(mS)\tRate(%%)\n");

	for (i = STATICS_NUMBER - 1; i >= 0; i--) {
		if ((max_wakelock_list[i].ws != NULL)
			&& (max_wakelock_list[i].max_ws_name[0] != 0)) {
			max_ws_rate = ktime_compare(wall_delta,
					max_wakelock_list[i].max_ws_time) >= 0 ? ktime_to_ms(
					max_wakelock_list[i].max_ws_time) * 100 /
				ktime_to_ms(wall_delta) : 100;
			buf_offset += scnprintf(buf + buf_offset, 200, "%s\t%lld\t%d\n",
					max_wakelock_list[i].max_ws_name,
					ktime_to_ms(max_wakelock_list[i].max_ws_time), max_ws_rate);
		}
	}

	mutex_unlock(&max_wakelock_list_lock);
	return buf_offset;
}
static ssize_t active_max_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf,
	size_t count)
{
	char reset_string[] = "reset";

	if (!((count == strlen(reset_string)) || ((count == strlen(reset_string) + 1)
				&& (buf[count - 1] == '\n')))) {
		return count;
	}

	if (strncmp(buf, reset_string, strlen(reset_string)) != 0) {
		return count;
	}

	active_max_reset();
	return count;
}

static void dump_active_max()
{
	int srcuidx, i, j;
	int max_ws_rate;
	ktime_t cur_ws_total_time, cur_ws_total_time_backup;
	ktime_t wall_delta;
	unsigned long flags;
	struct wakeup_source *ws;
	struct list_head *ws_listhead;

	get_ws_listhead(&ws_listhead);
	mutex_lock(&max_wakelock_list_lock);
	memset(max_wakelock_list, 0, sizeof(max_wakelock_list));
	wakeup_srcu_read_lock(&srcuidx);
	list_for_each_entry_rcu(ws, ws_listhead, entry) {
		ktime_t active_time;
		ktime_t now = ktime_get();
		spin_lock_irqsave(&ws->lock, flags);
		cur_ws_total_time = ws->total_time;

		if (ws->active) {
			active_time = ktime_sub(now, ws->last_time);
			cur_ws_total_time = ktime_add(ws->total_time, active_time);
		}

		cur_ws_total_time_backup = ws->total_time_backup;
		spin_unlock_irqrestore(&ws->lock, flags);

		if (ktime_compare(cur_ws_total_time, cur_ws_total_time_backup) >= 0) {
				for (i = 0; i < STATICS_NUMBER; i++) {
					if (ktime_compare(ktime_sub(cur_ws_total_time, cur_ws_total_time_backup),
							max_wakelock_list[i].max_ws_time) > 0) {
						for (j = STATICS_NUMBER - 1 ; j >= i + 1; j--) {
							max_wakelock_list[j].ws = max_wakelock_list[j - 1].ws;
							max_wakelock_list[j].max_ws_time = max_wakelock_list[j - 1].max_ws_time;
						}

						max_wakelock_list[i].ws = ws;
						max_wakelock_list[i].max_ws_time = ktime_sub(cur_ws_total_time,
								cur_ws_total_time_backup);
						break;
				}
			}
		}
	}
	for (i = 0; i < STATICS_NUMBER; i++) {
		if ((max_wakelock_list[i].ws != NULL)
			&& (max_wakelock_list[i].ws->name != NULL)) {
			scnprintf(max_wakelock_list[i].max_ws_name, NAME_SIZE - 1, "%s",
				max_wakelock_list[i].ws->name);
		}
	}

	wakeup_srcu_read_unlock(srcuidx);

	wall_delta = ktime_sub(ktime_get(), active_max_reset_time);
	PR_INFO("active_wakeup_source_state:");
	PR_INFO("--->Name     Time(mS)     Rate(%%)\n");

	for (i = STATICS_NUMBER - 1; i >= 0; i--) {
		if ((max_wakelock_list[i].ws != NULL)
			&& (max_wakelock_list[i].max_ws_name[0] != 0)) {
			max_ws_rate = ktime_compare(wall_delta,
					max_wakelock_list[i].max_ws_time) >= 0 ? ktime_to_ms(
					max_wakelock_list[i].max_ws_time) * 100 /
				ktime_to_ms(wall_delta) : 100;
			PR_INFO("--->%s     %lld     %d\n", max_wakelock_list[i].max_ws_name,
				ktime_to_ms(max_wakelock_list[i].max_ws_time), max_ws_rate);
		}
	}
	mutex_unlock(&max_wakelock_list_lock);
}


static struct kobj_attribute active_max = __ATTR_RW(active_max);

/*-------------------Function: add for statistics TOP4 active wakeup source -------------------*/



/* -------------Function: The node for statistics AP/MODEM/VDDMIN suspendratio ------------- */

#define MD_SLEEP_INFO_SMEM_OFFEST (4)
extern u64 ap_pd_count;
extern u64 ap_slp_duration;
extern u64 spm_26M_off_count;
extern u64 spm_26M_off_duration;
static DEFINE_MUTEX(rpm_master_stats_lock);
#if defined(CONFIG_MTK_ECCCI_DRIVER)
struct md_sleep_status md_data;
#endif

static void update_rpm_state() {
#if defined(CONFIG_MTK_ECCCI_DRIVER)
	u32 *share_mem = NULL;
	mutex_lock(&rpm_master_stats_lock);
	share_mem = (u32 *)get_smem_start_addr(MD_SYS1 , SMEM_USER_LOW_POWER , NULL);
	if (share_mem != NULL){
		share_mem = share_mem + MD_SLEEP_INFO_SMEM_OFFEST;
		memset(&md_data, 0, sizeof(struct md_sleep_status));
		memcpy(&md_data , share_mem , sizeof(struct md_sleep_status));
	}
	PR_INFO("vmin:%x:%llx	APSS:%x:%llx	MPSS:%x:%llx\n",
	spm_26M_off_count, PCM_TICK_TO_MILLI_SEC(spm_26M_off_duration), ap_pd_count,
	PCM_TICK_TO_MILLI_SEC(ap_slp_duration),
	md_data.sleep_cnt,
	PCM_TICK_TO_MILLI_SEC(md_data.sleep_time));
	mutex_unlock(&rpm_master_stats_lock);
#else
	return;
#endif
}

static ssize_t oplus_rpm_master_stats_show(struct kobject *kobj , struct kobj_attribute *attr , char *buf)
{
	/* dump sleep info */
#if defined(CONFIG_MTK_ECCCI_DRIVER)
	u32 len = 0;
	update_rpm_state();
	len = snprintf(buf , 200 , "vmin:%x:%llx\nAPSS:%x:%llx\nMPSS:%x:%llx\n",
	spm_26M_off_count, PCM_TICK_TO_MILLI_SEC(spm_26M_off_duration), ap_pd_count,
	PCM_TICK_TO_MILLI_SEC(ap_slp_duration),
	md_data.sleep_cnt,
	PCM_TICK_TO_MILLI_SEC(md_data.sleep_time));
	return len;
#else
	return 0;
#endif
}

static struct kobj_attribute oplus_rpm_master_stats = __ATTR_RO(oplus_rpm_master_stats);

/* -------------Function: The node for statistics AP/MODEM/VDDMIN suspendratio ------------- */



/*----------------------------- define statistics node------------------------------------*/

static struct attribute *attrs[] = {
	&ap_resume_reason_stastics.attr,
	&wakeup_stastisc_reset.attr,
	&kernel_time.attr,
	&active_max.attr,
	&clkstate_stastics_reset.attr,
	&clk_sleep_state_stastics.attr,
	&oplus_rpm_stats_statics.attr,
	&oplus_rpm_stats_statics_reset.attr,
	&oplus_rpm_master_stats.attr,
	&all_ap_resume_reason_stastics.attr,
	&all_resume_reason_stastics_reset.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

/*----------------------------- define statistics node------------------------------------*/



/*---------------------Function: Add for deal with screen ON/OFF event --------------------- */

struct timespec64 print_utc_time(char *annotation)
{
	struct timespec64 ts;
	struct rtc_time tm;

	getnstimeofday64(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("%s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	return ts;
}


static void screen_off_info_record()
{
	statisticstime_start = print_utc_time("Start clear power info...");
	update_rpm_state();/*print vddmin/ap/modem state*/
	kernel_time_reset();/*clear wakeup source states when screen off*/
	active_max_reset();/*clear wakeup source states when screen off*/
	wakeup_reasons_clear(WS_CNT_ALL);/*clear wakeup states when screen off*/
	clk_sleep_state_clear(CLK_CNT_CLEAR);/*clear clk states when screen off*/
	oplus_rpm_stats_statics_clear(); /*clear rpm states when screen off*/
	return;
}

static void screen_on_info_record()
{
	print_utc_time("Start dump power info...");
	update_rpm_state();/*print vddmin/ap/modem state*/
	if ((ts_start.tv_sec != 0)
		&& ((ts_end.tv_sec - ts_start.tv_sec) >=
			PRINT_THREAD_CONTROL)) { /*dump power info when screen off time >=PRINT_THREAD_CONTROL*/
		screen_off_time = ts_end.tv_sec - ts_start.tv_sec;

		if (statisticstime_start.tv_sec != 0) {
			real_statistics_time = ts_end.tv_sec - statisticstime_start.tv_sec;

		} else {
			real_statistics_time = 0;
		}

		dump_active_max();
		wakeup_reasons_print(PRINT_ALL_STATIC_INFO, print_module_info);
		dump_clk_state(PRINT_ALL_STATIC_INFO, print_all_detail_info);
		dump_rpmh_state();

	} else {
		screen_off_time = 0;
		real_statistics_time = 0;
	}

	return;
}

static DECLARE_DELAYED_WORK(clear_power_info, screen_off_info_record);
static DECLARE_DELAYED_WORK(dump_power_info, screen_on_info_record);

static void dump_power_info_control(char *annotation)
{
	if (!strncmp(annotation, "SCREEN_OFF", sizeof("SCREEN_OFF"))) {
		ts_start = print_utc_time("SCREEN_OFF");
		schedule_delayed_work(&clear_power_info,
			5 * HZ); /*when screen off,delay 5s(kernel time) to clear power info*/
		cancel_delayed_work_sync(&dump_power_info);

	} else if (!strncmp(annotation, "SCREEN_ON", sizeof("SCREEN_ON"))) {
		ts_end = print_utc_time("SCREEN_ON");
		schedule_delayed_work(&dump_power_info, 0 * HZ);
		cancel_delayed_work_sync(&clear_power_info);
	}

	return;
}


static bool screen_off_flag = true;
static bool screen_on_flag = true;

static int ws_fb_notify_callback(struct notifier_block *nb, unsigned long val,
	void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;

	if (val != FB_EVENT_BLANK) {
		return 0;
	}

	if (evdata && evdata->data && val == FB_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
		PR_INFO("val=%ld,blank=%d\n", val, blank);

		switch (blank) {
		case FB_BLANK_POWERDOWN: /*screen off*/
			screen_off_flag = true;
			if (screen_on_flag && screen_off_flag) {
				dump_power_info_control("SCREEN_OFF");
			}
			screen_on_flag = false;
			break;

		case FB_BLANK_UNBLANK:   /*screen on*/
			screen_on_flag = true;
			if (screen_on_flag && screen_off_flag) {
				dump_power_info_control("SCREEN_ON");
			}
			screen_off_flag = false;
			break;

		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block ws_fb_notify_block = {
	.notifier_call =  ws_fb_notify_callback,
};

/*---------------------Function: Add for deal with screen ON/OFF event --------------------- */



/*------- Function: When suspend prepare failed, printk active wakeup source every 5000ms------- */

#define wakelock_printk_interval_ms  (5*1000)
static int wakelock_printk_repeat = 0;

static int __pm_print_active_wakeup_sources(void)
{
	struct wakeup_source *ws;
	int srcuidx, active = 0;
	struct wakeup_source *last_activity_ws = NULL;
	struct list_head *ws_listhead;

	get_ws_listhead(&ws_listhead);
	wakeup_srcu_read_lock(&srcuidx);
	list_for_each_entry_rcu(ws, ws_listhead, entry) {
		if (ws->active) {
			PR_INFO("%s, %ld, %ld\n", ws->name,
				ws->active_count, ktime_to_ms(ws->total_time));
			active = 1;

		} else if (!active &&
			(!last_activity_ws ||
				ktime_to_ns(ws->last_time) >
				ktime_to_ns(last_activity_ws->last_time))) {
			last_activity_ws = ws;
		}
	}

	if (!active && last_activity_ws)
		PR_INFO("%s, %ld, %ld\n", last_activity_ws->name,
			last_activity_ws->active_count, ktime_to_ms(last_activity_ws->total_time));

	wakeup_srcu_read_unlock(srcuidx);

	return 0;
}

static void wakelock_printk(struct work_struct *w)
{
	struct delayed_work *dwork = container_of(w, struct delayed_work, work);

	if (__pm_print_active_wakeup_sources()) {
		return;
	}

	if (wakelock_printk_repeat) {
		schedule_delayed_work(dwork, msecs_to_jiffies(wakelock_printk_interval_ms));
	}
}
static DECLARE_DELAYED_WORK(wakelock_printk_work, wakelock_printk);

static void wakelock_printk_control(int on)
{
	if (on) {
		wakelock_printk_repeat = 1;
		schedule_delayed_work(&wakelock_printk_work,
			msecs_to_jiffies(wakelock_printk_interval_ms));

	} else {
		wakelock_printk_repeat = 0;
		cancel_delayed_work_sync(&wakelock_printk_work);
	}
}

static int wakelock_printk_notifier(struct notifier_block *nb,
	unsigned long event, void *unused)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		wakelock_printk_control(0);
		break;

	case PM_POST_SUSPEND:
		wakelock_printk_control(1);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block wakelock_printk_pm_nb = {
	.notifier_call = wakelock_printk_notifier,
	.priority = INT_MAX,
};

static int __init wakelock_printk_function_init(void)
{
	int ret;
	ret = register_pm_notifier(&wakelock_printk_pm_nb);

	if (ret) {
		PR_INFO("%s wakelock_printk_pm_nb error %d\n", ret);
		return -1;
	}

	wakelock_printk_control(1);

	return 0;
}

/*------- Function: When suspend prepare failed, printk active wakeup source every 5000ms------- */



/*-------------------------------------initialization----------------- ------------------ */

static struct kobject *wakelock_profiler;

static int __init wakelock_statistics_function_init(void)
{
	int retval;

	active_max_reset_time = ktime_set(0, 0);
	wakelock_profiler = kobject_create_and_add("wakelock_profiler", kernel_kobj);

	if (!wakelock_profiler) {
		printk(KERN_WARNING "[%s] failed to create a sysfs kobject\n",
			__func__);
		return 1;
	}

	retval = sysfs_create_group(wakelock_profiler, &attr_group);

	if (retval) {
		kobject_put(wakelock_profiler);
		printk(KERN_WARNING "[%s] failed to create a sysfs group %d\n",
			__func__, retval);
	}

	retval = fb_register_client(&ws_fb_notify_block);

	if (retval) {
		printk("%s error: register notifier failed!\n", __func__);
	}

	return 0;
}

/*-------------------------------------initialization----------------- ------------------ */

static int __init oplus_wakelock_profile_init(void)
{
	wakelock_statistics_function_init();
	wakelock_printk_function_init();
	pmic_irq_count_function_init();
	PR_INFO("oplus_wakelock_profile probed!\n");
	return 0;
}

late_initcall(oplus_wakelock_profile_init);
