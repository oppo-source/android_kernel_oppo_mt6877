#ifdef OPLUS_FEATURE_POWERINFO_STANDBY
#code deprecated ... start (Todo : delete this code section)

ifeq ($(CONFIG_OPLUS_POWER_MTK),y)
	obj-$(CONFIG_OPLUS_WAKELOCK_PROFILER)	+= oplus_wakelock_profiler_mtk.o
else ifeq ($(CONFIG_OPLUS_POWER_QCOM),y)
	obj-$(CONFIG_OPLUS_WAKELOCK_PROFILER)    += oplus_wakelock_profiler_qcom.o
else ifeq ($(CONFIG_OPLUS_WAKELOCK_PROFILER),y)
	obj-y                                   += oplus_wakelock_profiler.o
	obj-$(CONFIG_OPLUS_RBSC_MONITOR) += RBSC/oplus_RBSC_monitor.o
	obj-$(CONFIG_OPLUS_RBSC_MONITOR) += RBSC/oplus_RBSC_handler.o
endif

$(info wakelock MTK_PROJECT is $(MTK_PROJECT))
ifeq ($(MTK_PROJECT),oplus6877)
KBUILD_CFLAGS += -DCONFIG_POWER_6877_VERSION
KBUILD_CPPFLAGS += -DCONFIG_POWER_6877_VERSION
endif
$(info wakelock CONFIG_POWER_6877_VERSION is $(CONFIG_POWER_6877_VERSION))

#endif /* OPLUS_FEATURE_POWERINFO_STANDBY */
