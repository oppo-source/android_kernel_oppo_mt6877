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

//threashold info
#define battery_array_size (3)
#define DEFAULT_TIME_DIFF_THRESHOLD (1200) //20min
#define DEFAULT_KERNEL_RATIO_THRESHOLD (92)
#define DEFAULT_PC_TIME_DIFF_THRESHOLD (1000)
#define DEFAULT_VLOW_RATIO_THRESHOLD (0)  //aosd
#define DEFAULT_VMIN_RATIO_THRESHOLD (90)   //cxsd
#define DEFAULT_AVERAGE_CURRENT_RATIO_THRESHOLD (60)

//info mode=1;minidump mode=2; fulldump mode=3
#define DEFAULT_RBSC_ENABLE 0

#define printk_node(node, tm_last, tm_now) do{rtc_time_to_tm((node)->last_wall_time_last.tv_sec, &tm_last);rtc_time_to_tm((node)->now_wall_time_now.tv_sec, &tm_now);dsmsg("is_valid=%d, capacity_last=%d, capacity_now=%d, capacity_delta=%d, last_wall_time_last=%d-%02d-%02d %02d:%02d:%02d(UTC),  now_wall_time_now=%d-%02d-%02d %02d:%02d:%02d(UTC), wall_time_delta=%dS, kernel_time_last=%d, kernel_time_now=%d, kernel_time_delta=%d, kernel_suspend_ratio=%d%%, aosd_time_last=%d, aosd_time_now=%d, aosd_time_delta=%d, aosd_suspend_ratio=%d%%, cxsd_time_last=%d, cxsd_time_now=%d, cxsd_time_delta=%d, cxsd_suspend_ratio=%d%%, avg_power_avg=%d",(node)->is_valid, (node)->capacity_last, (node)->capacity_now, (node)->capacity_delta,tm_last.tm_year + 1900, tm_last.tm_mon + 1, tm_last.tm_mday, tm_last.tm_hour, tm_last.tm_min, tm_last.tm_sec,tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec,(node)->wall_time_delta, (node)->kernel_time_last.tv_sec, (node)->kernel_time_now.tv_sec, (node)->kernel_time_delta, (node)->kernel_suspend_ratio, (node)->aosd_time_last, (node)->aosd_time_now, (node)->aosd_time_delta, (node)->aosd_suspend_ratio, (node)->cxsd_time_last, (node)->cxsd_time_now, (node)->cxsd_time_delta, (node)->cxsd_suspend_ratio, (node)->avg_power_avg);}while(0)

static struct kobject *RBSC_config;
static int g_RBSC_enable = DEFAULT_RBSC_ENABLE;
static int g_kernel_ratio_threshold = DEFAULT_KERNEL_RATIO_THRESHOLD;
static int g_vlow_ratio_threshold = DEFAULT_VLOW_RATIO_THRESHOLD;
static int g_vmin_ratio_threshold = DEFAULT_VMIN_RATIO_THRESHOLD;
static int g_avrage_current_threshold = DEFAULT_AVERAGE_CURRENT_RATIO_THRESHOLD;
static int g_screenoff_timeout_second_threshold = DEFAULT_TIME_DIFF_THRESHOLD;

extern int get_rpmh_deep_sleep_info(u64 *aosd, u64 *cxsd);
extern int get_oplus_display_power_status(void);

static bool function_enable = false;
static bool is_screenoff_state_valid = false; //aod:invalid  screen-on:invalid  screen-off:valid
static struct timespec screen_off_wall_time;
static int battery_array_index = 0;
static debug_battery_stats_desc battery_arry[battery_array_size];
static DEFINE_SPINLOCK(battery_arry_lock);

static inner_temp_enable = false;
void deepsleep_issue_screenoff(void)
{
	//battery_array_index = 0;
	//function_enable = true;
	dsmsg("");
	inner_temp_enable = true;
	getnstimeofday(&screen_off_wall_time);

	if (g_RBSC_enable > FULL_DUMP_MODE)
		oplus_save_clk_regulator_info_start();

	return;
}

void deepsleep_issue_screenon(void)
{
	unsigned long flags;
	dsmsg("");
	function_enable = false;
	inner_temp_enable = false;
	spin_lock_irqsave(&battery_arry_lock, flags);
	battery_array_index = 0;
	memset(battery_arry, 0, sizeof(battery_arry));
	spin_unlock_irqrestore(&battery_arry_lock, flags);

/*For debug , if in test mode ,triger RBSC_issue_handler when screen on  */
	if (g_RBSC_enable > FULL_DUMP_MODE)
		oplus_save_clk_regulator_info_end();

	if (g_RBSC_enable == FULL_DUMP_TEST_MODE){
		RBSC_issue_handler(FULL_DUMP_MODE,"vlow");
	}else if(g_RBSC_enable == MINI_DUMP_TEST_MODE){
		RBSC_issue_handler(MINI_DUMP_MODE,"vlow");
	}else if(g_RBSC_enable == DEBUG_INFO_TEST_MODE){
		RBSC_issue_handler(DEBUG_INFO_MODE,"vlow");
	}

	
	return;
}

static bool screenoff_state_ever_invalid_data = false;
static void screenoff_state_ever_invalid(int screen_display_state)
{
	if(screen_display_state == 0) {//0 : screen on
		screenoff_state_ever_invalid_data = true;
		return;
	}
	if(screen_display_state == 1) {//1 : screen-off and aod state
		screenoff_state_ever_invalid_data = true;
		return;
	}
	return;
}

void screenoff_state_valid_set(int screen_display_state)
{
	struct timespec wall_time, kerneltime;
	struct rtc_time tm_now;

	getnstimeofday(&wall_time);
	rtc_time_to_tm(wall_time.tv_sec, &tm_now);

	jiffies_to_timespec(get_jiffies_64(), &kerneltime);

	dsmsg("tm_now=%d-%02d-%02d %02d:%02d:%02d(UTC), kerneltime=%d, screen_display_state=%d", tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday, tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, kerneltime.tv_sec, screen_display_state);

	screenoff_state_ever_invalid(screen_display_state);
	if(screen_display_state == 0) {//0 : screen on
		is_screenoff_state_valid = false;
		return;
	}
	if(screen_display_state == 1) {//1 : screen-off and aod state
		is_screenoff_state_valid = false;
		return;
	}
	if(screen_display_state == 2) {//2 : screen-off state
		is_screenoff_state_valid = true;
		return;
	}
	return;
}
EXPORT_SYMBOL(screenoff_state_valid_set);


static void printk_battery_arry()
{
	int i=0;
	struct rtc_time tm_last;
	struct rtc_time tm_now;
	dsmsg("show battery_arry list : ");
	for(i=0; i<battery_array_size; i++) {
		printk_node(&battery_arry[i], tm_last, tm_now);
	}
}

static bool is_issue_node(debug_battery_stats_desc * node)
{
	if(node->is_valid == false) {
		return false;
	}
	if(node->kernel_suspend_ratio <= g_kernel_ratio_threshold) {
		return false;
	}
	if(node->aosd_suspend_ratio <= g_vlow_ratio_threshold) {
		return false;
	}
	if(node->cxsd_suspend_ratio <= g_vmin_ratio_threshold) {
		return false;
	}
	if(node->avg_power_avg >= g_avrage_current_threshold) {
		return false;
	}
	return true;
}


static void RBSC_issue_judgement()
{
	int i=0;
	unsigned long flags;
	dsmsg("");
	printk_battery_arry();
	spin_lock_irqsave(&battery_arry_lock, flags);
	for(i=0; i<battery_array_size; i++) {
		if(!is_issue_node(&battery_arry[i])){
			spin_unlock_irqrestore(&battery_arry_lock, flags);
			return;
		}
		if(!is_clk_dump_start_record()){
			oplus_save_clk_regulator_info_start();
		}
	}
	spin_unlock_irqrestore(&battery_arry_lock, flags);

	dsmsg("pc_powerleak");
	RBSC_issue_handler(g_RBSC_enable, "vlow");
}

static debug_battery_stats_desc * get_new_node()
{
	debug_battery_stats_desc * temp = &battery_arry[battery_array_index];
	battery_array_index = (battery_array_index + 1) % battery_array_size;
	return temp;
}

void battery_capacity_change_notify(int last, int cur)
{
	static int capacity_last = -1;
	static struct timespec last_wall_time_last;
	static struct timespec kernel_time_last;
	static u64 aosd_time_last;
	static u64 cxsd_time_last;
	debug_battery_stats_desc * node = NULL;

	struct rtc_time tm_last;
	struct rtc_time tm_now;

	static u64 aosd_time_temp;
	static u64 cxsd_time_temp;

	unsigned long flags;

	dsmsg("function_enable=%d, inner_temp_enable=%d, is_screenoff_state_valid=%d, screenoff_state_ever_invalid_data=%d", function_enable, inner_temp_enable, is_screenoff_state_valid, screenoff_state_ever_invalid_data);
	if((last == -1) || (!function_enable)){ //only update last info, but do not anything
		capacity_last = cur;
		getnstimeofday(&last_wall_time_last);
		jiffies_to_timespec(get_jiffies_64(), &kernel_time_last);
		get_rpmh_deep_sleep_info(&aosd_time_last, &cxsd_time_last);
		if(!is_screenoff_state_valid) {
			screenoff_state_ever_invalid_data = true;
		} else {
			screenoff_state_ever_invalid_data = false;
		}
		return;
	}

	get_rpmh_deep_sleep_info(&aosd_time_temp, &cxsd_time_temp);

	spin_lock_irqsave(&battery_arry_lock, flags);
	node = get_new_node();
	if(!node){
		dsmsg("node is null pointer, return\n");
		spin_unlock_irqrestore(&battery_arry_lock, flags);
		return;
	}

	node->is_valid = true;

	node->capacity_last = capacity_last;
	node->last_wall_time_last.tv_sec = last_wall_time_last.tv_sec;
	node->last_wall_time_last.tv_nsec = last_wall_time_last.tv_nsec;
	node->kernel_time_last.tv_sec = kernel_time_last.tv_sec;
	node->kernel_time_last.tv_nsec = kernel_time_last.tv_nsec;
	node->aosd_time_last = aosd_time_last;
	node->cxsd_time_last = cxsd_time_last;

	node->capacity_now = cur;
	getnstimeofday(&node->now_wall_time_now);
	jiffies_to_timespec(get_jiffies_64(), &node->kernel_time_now);
	node->aosd_time_now = aosd_time_temp;
	node->cxsd_time_now = cxsd_time_temp;


	//node->capacity_delta = node->capacity_now - node->capacity_last;
	node->capacity_delta = node->capacity_last - node->capacity_now;
	node->wall_time_delta = node->now_wall_time_now.tv_sec - node->last_wall_time_last.tv_sec ;
	node->kernel_time_delta = node->kernel_time_now.tv_sec - node->kernel_time_last.tv_sec ;
	node->aosd_time_delta = node->aosd_time_now - aosd_time_last;
	node->cxsd_time_delta = node->cxsd_time_now - cxsd_time_last;

	node->kernel_suspend_ratio =  100 - node->kernel_time_delta * 100 / node->wall_time_delta;
	node->aosd_suspend_ratio =  node->aosd_time_delta * 100 / 1000  / node->wall_time_delta;
	node->cxsd_suspend_ratio =  node->cxsd_time_delta * 100 / 1000  / node->wall_time_delta;
	node->avg_power_avg = node->capacity_delta * (4200 / 100) *  3600 / node->wall_time_delta; //todo : 4200 is battery size, adjust to get this info auto.

	if(node->capacity_delta <= 0) {
		node->is_valid = false;
	}
	if(node->kernel_time_delta < 0) {
		node->is_valid = false;
	}
	if((node->wall_time_delta < 0) || (node->wall_time_delta > 3600 * 24 * 360 * 2)) {
		node->is_valid = false;
	}
	if((node->aosd_time_delta < 0) || (node->cxsd_time_delta < 0)) {
		node->is_valid = false;
	}
	if(!is_screenoff_state_valid) {
		screenoff_state_ever_invalid_data = true;
		node->is_valid = false;
	} else {
		if(screenoff_state_ever_invalid_data) {
			node->is_valid = false;
		} else {
			node->is_valid = true;
		}
		screenoff_state_ever_invalid_data = false;
	}
	//update last info to latest
	capacity_last = node->capacity_now;
	last_wall_time_last.tv_sec = node->now_wall_time_now.tv_sec;
	last_wall_time_last.tv_nsec = node->now_wall_time_now.tv_nsec;
	kernel_time_last.tv_sec = node->kernel_time_now.tv_sec;
	kernel_time_last.tv_nsec = node->kernel_time_now.tv_nsec;
	aosd_time_last = node->aosd_time_now;
	cxsd_time_last = node->cxsd_time_now;
	spin_unlock_irqrestore(&battery_arry_lock, flags);

	printk_node(node, tm_last, tm_now);
	RBSC_issue_judgement();
}

static int dsdebug_psy_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	int ret = 0;
	static int last_battery_capacity = -1;
	union power_supply_propval cur_value;
	struct power_supply *battery_psy = NULL;

	if(!inner_temp_enable) {
		dsmsg("inner_temp_enable=%d, return\n", inner_temp_enable);
		return 0;
	}

	//dsmsg("some power supply property change notify\n");
	battery_psy = power_supply_get_by_name("battery");
	if(battery_psy == NULL) {
		dsmsg("power_supply_get_by_name fail\n");
	}

	if ((struct power_supply *)ptr != battery_psy ||
				evt != PSY_EVENT_PROP_CHANGED) {
		return 0;
	}

	//dsmsg("battery power supply property change notify\n");
	ret = power_supply_get_property(battery_psy,
			POWER_SUPPLY_PROP_CAPACITY, &cur_value);
	if (ret) {
		dsmsg("battery capacity get error, ret is %d\n", ret);
		return ret;
	}
	dsmsg("battery cur capacity is %d\n", cur_value.intval);

	if (is_clk_dump_start_record())
		oplus_get_clk_regulater_info();

	if(cur_value.intval != last_battery_capacity) { //battery capacity change notify
		battery_capacity_change_notify(last_battery_capacity, cur_value.intval);
		last_battery_capacity = cur_value.intval;
	}
	return ret;
}

static struct notifier_block psy_nb;
static int psy_notify_reg() {
	int err;
	psy_nb.priority = 0;
	psy_nb.notifier_call = dsdebug_psy_event_changed;
	err = power_supply_reg_notifier(&psy_nb);
	return err;
}

static void psy_notify_unreg() {
	power_supply_unreg_notifier(&psy_nb);
}


//pm suspend/resume call back module  --start
static void RBSC_monitor_pm_resume_work()
{
	struct timespec walltime;
	dsmsg("");
	if(!inner_temp_enable) {
		dsmsg("inner_temp_enable=%d, return\n", inner_temp_enable);
		return;
	}

	/* screen off time condition */
	getnstimeofday(&walltime);
	if(walltime.tv_sec <= screen_off_wall_time.tv_sec) {
		dsmsg("walltime diff is -1 seconds");
		return;
	}

	dsmsg("wall time diff is %llu seconds", walltime.tv_sec - screen_off_wall_time.tv_sec);
	if((walltime.tv_sec - screen_off_wall_time.tv_sec) <= g_screenoff_timeout_second_threshold) {
		dsmsg("screen off wall time is %ds, less than %ds, ignore it",
				walltime.tv_sec - screen_off_wall_time.tv_sec, g_screenoff_timeout_second_threshold);
		return;
	}

	if(!function_enable) { //screen off time out, enable this feature.
		battery_array_index = 0;
		function_enable = true;
	}
}

static int RBSC_monitor_notifier(struct notifier_block *nb,
				  unsigned long event, void *unused)
{
	int power_mode;

	switch (event) {
	case PM_POST_SUSPEND:
		dsmsg("PM_POST_SUSPEND, g_RBSC_enable=%d", g_RBSC_enable);
		if (g_RBSC_enable != 0)
			RBSC_monitor_pm_resume_work();
		break;
	case PM_POST_RESTORE:
		dsmsg("PM_POST_RESTORE, g_RBSC_enable=%d", g_RBSC_enable);
		if(g_RBSC_enable != 0) {
			power_mode = get_oplus_display_power_status();
			if(power_mode == 0) {
				deepsleep_issue_screenon();
			}
			screenoff_state_ever_invalid(power_mode);
		}
		break;
	}

	if ((g_RBSC_enable != 0) && is_clk_dump_start_record())
		oplus_get_clk_regulater_info();

	return NOTIFY_DONE;
}

static struct notifier_block deepsleep_debug_pm_nb = {
	.notifier_call = RBSC_monitor_notifier,
	.priority = INT_MAX,
};
//pm suspend/resume call back module  --end
static ssize_t RBSC_monitor_config_parameter_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int buf_offset = 0;
	buf_offset = sprintf(buf, "enable=%d,kernel>%d,vmin>%d,vlow>%d,current>%d,time_diff>%d\n", \
		g_RBSC_enable, g_kernel_ratio_threshold, g_vmin_ratio_threshold, g_vlow_ratio_threshold, g_avrage_current_threshold, g_screenoff_timeout_second_threshold);

	return buf_offset;
}

static ssize_t RBSC_monitor_config_parameter_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	dsmsg("%s", buf);

	if(buf[count-1] != '\n'){
		dsmsg("ERROR FORMAT!USAGE: enable=0,kernel>92,vmin>90,vlow>0,current>55,time_diff>3600");
		return count;
	}

	ret = sscanf(buf, "enable=%d,kernel>%d,vmin>%d,vlow>%d,current>%d,time_diff>%d\n", \
		&g_RBSC_enable, &g_kernel_ratio_threshold, &g_vmin_ratio_threshold, &g_vlow_ratio_threshold, &g_avrage_current_threshold, &g_screenoff_timeout_second_threshold );

	if (ret != 6){
		dsmsg("ERROR FORMA! ret =%d, USAGE:  enable=0,kernel>92,vmin>90,vlow>0,current>55,time_diff>3600", ret);
		g_RBSC_enable = DEFAULT_RBSC_ENABLE;
		g_kernel_ratio_threshold = DEFAULT_KERNEL_RATIO_THRESHOLD;
		g_vlow_ratio_threshold = DEFAULT_VLOW_RATIO_THRESHOLD;
		g_vmin_ratio_threshold = DEFAULT_VMIN_RATIO_THRESHOLD;
		g_avrage_current_threshold = DEFAULT_AVERAGE_CURRENT_RATIO_THRESHOLD;
		g_screenoff_timeout_second_threshold = DEFAULT_TIME_DIFF_THRESHOLD;
	}

	return count;
}

static struct kobj_attribute RBSC_monitor_parameter = __ATTR_RW(RBSC_monitor_config_parameter);

static struct attribute *attrs[] = {
	&RBSC_monitor_parameter.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init oplus_RBSC_monitor_init(void)
{
    int ret;

	dsmsg("");
	memset(battery_arry, 0, sizeof(battery_arry));

	ret = psy_notify_reg();
	if (ret < 0) {
		dsmsg("psy_notify_reg error, ret=%d\n", ret);
		goto psy_notify_reg_error;
	}

	ret = register_pm_notifier(&deepsleep_debug_pm_nb);
	if (ret) {
		dsmsg("deepsleep_debug_pm_nb error %d", ret);
		goto register_pm_notifier_error;
	}

	RBSC_config = kobject_create_and_add("RBSC_monitor_config", kernel_kobj);
	if (!RBSC_config) {
		dsmsg("failed to create a sysfs kobject\n");
		goto register_pm_notifier_error;
	}
	ret = sysfs_create_group(RBSC_config, &attr_group);
	if (ret) {
		dsmsg("failed to create a sysfs group %d\n", ret);
		goto sysfs_create_group_error;
	}
	dsmsg("RBSC_monitor_node_init done");

	return 0;

sysfs_create_group_error:
	kobject_put(RBSC_config);

register_pm_notifier_error:
	psy_notify_unreg();

psy_notify_reg_error:
	return 0;
}


late_initcall(oplus_RBSC_monitor_init);