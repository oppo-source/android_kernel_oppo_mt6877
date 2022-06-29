/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
****************************************************************/
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/syscore_ops.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/sched/signal.h>
#include <linux/pm_wakeup.h>
#include <linux/mutex.h>
#include <soc/qcom/watchdog.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include "oplus_RBSC_monitor.h"
#include "oplus_RBSC_handler.h"

#define DUMP_PATH "/data/dump_clk_regulater.txt"

extern void oplus_switch_fulldump(int on);
extern void oplus_show_regulator_list(void);
extern void oplus_clock_debug_print_enabled(void);
//extern void oplus_bcm_client_read(void);

static int g_is_record_start = 0;

int oplus_save_clk_regulator_info_start(void){
	g_is_record_start = 1;
	return 0;
}

void oplus_save_clk_regulator_info_end(void){
	g_is_record_start = 0;
}

int is_clk_dump_start_record(void){
	return g_is_record_start;
}

void oplus_get_clk_regulater_info(void)
{
//	oplus_bcm_client_read();
	oplus_clock_debug_print_enabled();
	oplus_show_regulator_list();

}

void RBSC_issue_handler(int mode, char* issue_type)
{
	dsmsg("\n\n\n\n\n!!!!!!!!!!!!!!!!!!RBSC triggered, mode=%d, issue_type=%s!!!!!!!!!!!!!\n\n\n\n\n", mode, issue_type);
	oplus_show_regulator_list();

	if(mode == FULL_DUMP_MODE){
		dsmsg("PowerTeam: Trig full dump, issue tyep: %s\n\n", issue_type);
		oplus_switch_fulldump(true);
		msm_trigger_wdog_bite();

	}else if(mode == MINI_DUMP_MODE){
	//to do
		dsmsg("PowerTeam: Trig mini dump, issue tyep: %s\n\n", issue_type);
		oplus_switch_fulldump(false);
		msm_trigger_wdog_bite();
	}
}
EXPORT_SYMBOL(RBSC_issue_handler);