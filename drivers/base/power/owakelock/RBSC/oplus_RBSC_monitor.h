/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
****************************************************************/

#define FULL_DUMP_MODE 3
#define MINI_DUMP_MODE 2
#define DEBUG_INFO_MODE 1

#define FULL_DUMP_TEST_MODE 993
#define MINI_DUMP_TEST_MODE 992
#define DEBUG_INFO_TEST_MODE 991

typedef struct debug_battery_stats_desc {
	bool is_valid;

	int capacity_last;  //capacity info
	int capacity_now;   //capacity info
	int capacity_delta; //capacity info

	struct timespec  last_wall_time_last; //wal time info
	struct timespec  now_wall_time_now;   //wal time info
	int wall_time_delta;                  //wal time info--seconds unit

	struct timespec  kernel_time_last; //kernel time info
	struct timespec  kernel_time_now;  //kernel time info
	int kernel_time_delta;             //kernel time info--seconds unit
	int kernel_suspend_ratio;          //kernel info

	u64 aosd_time_last;     //aosd info--ms unit
	u64 aosd_time_now;      //aosd info--ms unit
	u64 aosd_time_delta;    //aosd info--ms unit
	int aosd_suspend_ratio; //aosd info
	
	u64 cxsd_time_last;     //cxsd info--ms unit
	u64 cxsd_time_now;      //cxsd info--ms unit
	u64 cxsd_time_delta;    //cxsd info--ms unit
	int cxsd_suspend_ratio; //cxsd info

	int avg_power_avg;      //powe info ---average power mA@4V,  <0 power loss(not charger),  >0 powe increase(charger)

} debug_battery_stats_desc;


#define opds_tag "RBSC"
#define dsmsg(format,...)         pr_info("%s : %s %d "format"\n", opds_tag, __func__, __LINE__, ##__VA_ARGS__)
