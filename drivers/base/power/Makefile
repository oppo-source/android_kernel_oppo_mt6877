# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_PM)	+= sysfs.o generic_ops.o common.o qos.o runtime.o wakeirq.o
obj-$(CONFIG_PM_SLEEP)	+= main.o wakeup.o wakeup_stats.o
obj-$(CONFIG_PM_TRACE_RTC)	+= trace.o
obj-$(CONFIG_PM_OPP)	+= opp/
obj-$(CONFIG_PM_GENERIC_DOMAINS)	+=  domain.o domain_governor.o
obj-$(CONFIG_HAVE_CLK)	+= clock_ops.o
obj-y	+= oplus_pwr_dbg/

ccflags-$(CONFIG_DEBUG_DRIVER) := -DDEBUG
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
obj-$(CONFIG_OPLUS_WAKELOCK_PROFILER) += owakelock/
#endif /* OPLUS_FEATURE_POWERINFO_STANDBY */
