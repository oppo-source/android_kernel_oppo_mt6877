/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include "imgsensor_hwcfg_custom.h"
struct IMGSENSOR_SENSOR_LIST *Oplusimgsensor_Sensorlist(void)
{
    struct IMGSENSOR_SENSOR_LIST *pOplusImglist = gimgsensor_sensor_list;

    printk("Oplusimgsensor_Sensorlist enter:\n");
    if (is_project(20131) || is_project(20133)
          || is_project(20255) || is_project(20257)) {
        pOplusImglist =  gimgsensor_sensor_list_20131;
        pr_info("gimgsensor_sensor_list_20131 Selected\n");
    } else if (is_project(20075)||is_project(20076)) {
        pOplusImglist =  gimgsensor_sensor_list_20075;
        printk("gimgsensor_sensor_list_20075 Selected\n");
    } else if (is_project(20181)||is_project(20355)) {
        pOplusImglist =  gimgsensor_sensor_list_20181;
        printk("gimgsensor_sensor_list_20181 Selected\n");
    }
    if ( is_project(20015) || is_project(20016)
        || is_project(20108) || is_project(20109)
        || is_project(20651) || is_project(20652)
        || is_project(20653) || is_project(20654) ) {
        pOplusImglist =  gimgsensor_sensor_list_20015;
        pr_debug("gimgsensor_sensor_list_20015 Selected\n");
    } else if ( is_project(20609) || is_project(0x2060A)
        || is_project(0x2060B) || is_project(0x20796)
        || is_project(0x206F0) || is_project(0x206FF)
        || is_project(0x2070C) || is_project(0x2070B)
        || is_project(0x2070E)) {
        pOplusImglist =  gimgsensor_sensor_list_20609;
        pr_debug("gimgsensor_sensor_list_20609 Selected\n");
    }
    return pOplusImglist;
}

struct IMGSENSOR_HW_CFG *Oplusimgsensor_Custom_Config(void)
{
    struct IMGSENSOR_HW_CFG *pOplusImgHWCfg = imgsensor_custom_config;

    if (is_project(21081)) {
        pOplusImgHWCfg = imgsensor_custom_config_21081;
        pr_info("imgsensor_custom_config_21081 senso Selected\n");
    }

    if (is_project(0x212A1)) {
        pOplusImgHWCfg = imgsensor_custom_config_212A1;
        pr_info("imgsensor_custom_config_212A1 senso Selected\n");
    } else if (is_project(21851) || is_project(21876)) {
        pOplusImgHWCfg = imgsensor_custom_config_21851;
        pr_info("imgsensor_custom_config_21851 senso Selected\n");
    }

    if (is_project(20131) || is_project(20133)
          || is_project(20255) || is_project(20257)) {
        pOplusImgHWCfg = imgsensor_custom_config_20131;
        pr_info("imgsensor_custom_config_20131 Selected\n");
    } else if (is_project(20075) || is_project(20076)) {
        pOplusImgHWCfg = imgsensor_custom_config_20075;
        pr_info("imgsensor_custom_config_20075 Selected\n");
    } else if (is_project(20181) || is_project(20355)) {
        pOplusImgHWCfg = imgsensor_custom_config_20181;
        pr_info("imgsensor_custom_config_20181 Selected\n");
    }
    if ( is_project(20015) || is_project(20016)
        || is_project(20108) || is_project(20109)
        || is_project(20651) || is_project(20652)
        || is_project(20653) || is_project(20654) ) {
        pOplusImgHWCfg = imgsensor_custom_config_20015;
        pr_info("imgsensor_custom_config_20015 Selected\n");
    } else if ( is_project(20609) || is_project(0x2060A)
        || is_project(0x2060B) || is_project(0x2070B)
        || is_project(0x2070E)) {
        pOplusImgHWCfg = imgsensor_custom_config_20609;
        pr_info("imgsensor_custom_config_20609 Selected\n");
    } else if ( is_project(0x20796) || is_project(0x206F0)
        || is_project(0x206FF) || is_project(0x2070C)) {
        pOplusImgHWCfg = imgsensor_custom_config_20796;
        pr_info("imgsensor_custom_config_20796 Selected\n");
    }

    return pOplusImgHWCfg;
}

enum IMGSENSOR_RETURN Oplusimgsensor_i2c_init(
        struct IMGSENSOR_SENSOR_INST *psensor_inst)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    struct IMGSENSOR_HW_CFG *pOplusImgHWCfg = imgsensor_custom_config;

    if (psensor_inst == NULL) {
        pr_info("Oplusimgsensor_i2c_init psensor_inst is NULL\n");
        return IMGSENSOR_RETURN_ERROR;
    }

    pOplusImgHWCfg = Oplusimgsensor_Custom_Config();
    ret = imgsensor_i2c_init(&psensor_inst->i2c_cfg,
                             pOplusImgHWCfg[psensor_inst->sensor_idx].i2c_dev);
    pr_info("[%s] sensor_idx:%d name:%s ret: %d\n",
        __func__,
        psensor_inst->sensor_idx,
        psensor_inst->psensor_list->name,
        ret);

    return ret;
}

struct IMGSENSOR_HW_POWER_SEQ *Oplusimgsensor_matchhwcfg_power(
        enum  IMGSENSOR_POWER_ACTION_INDEX  pwr_actidx)
{
    struct IMGSENSOR_HW_POWER_SEQ *ppwr_seq = NULL;
    pr_info("[%s] pwr_actidx:%d\n", __func__, pwr_actidx);
    if ((pwr_actidx != IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX)
        && (pwr_actidx != IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX)) {
        pr_info("[%s] Invalid pwr_actidx:%d\n", __func__, pwr_actidx);
        return NULL;
    }

    if (is_project(20131) || is_project(20133)
          || is_project(20255) || is_project(20257)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20131;
            pr_info("[%s] match sensor_power_sequence_20131\n", __func__);
        } else {
            pr_info("[%s] 20131 NOT Support MIPISWITCH\n", __func__);
        }
    } else if(is_project(21081)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_21081;
            pr_info("[%s] match sensor_power_sequence_21081\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX) {
            pr_info("[%s] enter for 21081 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    } else if(is_project(0x212A1)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_212A1;
            pr_info("[%s] match sensor_power_sequence_212A1\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX) {
            pr_info("[%s] enter for 212A1 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    } else if(is_project(21851) || is_project(21876)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_21851;
            pr_info("[%s] match sensor_power_sequence_21851\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX) {
            pr_info("[%s] enter for 21851 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    }
    else if (is_project(20075) || is_project(20076)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20075;
            pr_info("[%s] match sensor_power_sequence_20075\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            pr_info("[%s] enter for 20075 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    } else if (is_project(20181) || is_project(20355)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20181;
            pr_info("[%s] match sensor_power_sequence_20181\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            pr_info("[%s] enter for 20181 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    }
    if ( is_project(20015) || is_project(20016)
        || is_project(20108) || is_project(20109)
        || is_project(20651) || is_project(20652)
        || is_project(20653) || is_project(20654) ) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20015;
            pr_info("[%s] match sensor_power_sequence_20015\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            pr_info("[%s] enter for 20015 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    } else if ( is_project(20609) || is_project(0x2060A)
        || is_project(0x2060B) || is_project(0x20796)
        || is_project(0x206F0) || is_project(0x206FF)
        || is_project(0x2070C) || is_project(0x2070B)
        || is_project(0x2070E)) {
        if (pwr_actidx == IMGSENSOR_POWER_MATCHSENSOR_HWCFG_INDEX) {
            ppwr_seq = sensor_power_sequence_20609;
            pr_info("[%s] match sensor_power_sequence_20609\n", __func__);
        } else if (pwr_actidx == IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX){
            pr_info("[%s] enter for 20609 IMGSENSOR_POWER_MATCHMIPI_HWCFG_INDEX \n", __func__);
            return NULL;
        }
    }

    return ppwr_seq;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldoenable_power(
        struct IMGSENSOR_HW             *phw,
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    struct IMGSENSOR_HW_DEVICE *pdev = phw->pdev[IMGSENSOR_HW_ID_GPIO];
    pr_debug("[%s] sensor_idx:%d pwr_status: %d\n", __func__, sensor_idx, pwr_status);
    if (pdev->set != NULL) {
        if(is_project(21081) || (IMGSENSOR_HW_POWER_STATUS_ON == pwr_status)) {
            pmic_gpio_enable(pwr_status);
        }
    } else {
        pr_info("[%s] sensor_idx:%d pdev->set is ERROR\n", __func__, sensor_idx );
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int pmic_avdd[3][2] = {{3, 2800}, {4, 2900}, {6, 2800}};
    int avddIdx = sensor_idx > 1 ? 2 : sensor_idx;

    if(is_project(21081)) {
        return Oplusimgsensor_ldo_powerset_21081(sensor_idx,pin,pwr_status);
    } else if(is_project(0x212A1)) {
        return Oplusimgsensor_ldo_powerset_212A1(sensor_idx,pin,pwr_status);
    } else if(is_project(21851) || is_project(21876)) {
        return Oplusimgsensor_ldo_powerset_21851(sensor_idx,pin,pwr_status);
    } else {
       pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
               __func__, pmic_ldo_get_type(), pwr_status);
       if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
           pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
           pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
           if (pin == IMGSENSOR_HW_PIN_AVDD) {
               pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
           } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 2){
               pmic_ldo_set_voltage_mv(1, 1200);
           } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
               pmic_ldo_set_voltage_mv(2, 1100);
           } else {
               return IMGSENSOR_RETURN_ERROR;
           }
       } else {
           pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
           pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
           if (pin == IMGSENSOR_HW_PIN_AVDD) {
               pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
           } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 2){
               pmic_ldo_set_disable(1);
           } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
               pmic_ldo_set_disable(2);
           } else {
               return IMGSENSOR_RETURN_ERROR;
           }
       }
    }

    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_21081(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    int pmic_avdd_1[3][2] = {{3, 2900}, {4, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 1 ? 2 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd_1:%d %d", __func__, avddIdx, pmic_avdd_1[avddIdx][0], pmic_avdd_1[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd_1[avddIdx][0], pmic_avdd_1[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1050);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0) {
            pmic_ldo_set_voltage_mv(7, 2800);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd_1:%d %d", __func__, avddIdx, pmic_avdd_1[avddIdx][0], pmic_avdd_1[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd_1[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_disable(2);
        } else if (pin == IMGSENSOR_HW_PIN_AFVDD && sensor_idx == 0) {
            pmic_ldo_set_disable(7);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_212A1(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[4][2] = {{3, 2800}, {4, 2900}, {6, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 2 ? 3 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 2){
            pmic_ldo_set_voltage_mv(2, 1200);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1 || sensor_idx == 2)){
            pmic_ldo_set_disable(2);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}


enum IMGSENSOR_RETURN Oplusimgsensor_ldo_powerset_21851(
        enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
        enum   IMGSENSOR_HW_PIN          pin,
        enum   IMGSENSOR_HW_POWER_STATUS pwr_status)
{
    static int pmic_avdd[4][2] = {{3, 2800}, {4, 2900}, {6, 2800}, {6, 2800}};
    int avddIdx = sensor_idx > 2 ? 3 : sensor_idx;

    pr_debug("[%s] pmic_ldo_get_type:%d pwr_status:%d",
            __func__, pmic_ldo_get_type(), pwr_status);
    if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_voltage_mv(pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_voltage_mv(1, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 1){
            pmic_ldo_set_voltage_mv(2, 1100);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 2){
            pmic_ldo_set_voltage_mv(2, 1200);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    } else {
        pr_info("[%s] sensor_idx:%d pwr_status:%d pin:%d", __func__, sensor_idx, pwr_status, pin);
        pr_info("[%s] avddIdx:%d pmic_avdd:%d %d", __func__, avddIdx, pmic_avdd[avddIdx][0], pmic_avdd[avddIdx][1]);
        if (pin == IMGSENSOR_HW_PIN_AVDD) {
            pmic_ldo_set_disable(pmic_avdd[avddIdx][0]);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && sensor_idx == 0){
            pmic_ldo_set_disable(1);
        } else if (pin == IMGSENSOR_HW_PIN_DVDD && (sensor_idx == 1 || sensor_idx == 2)){
            pmic_ldo_set_disable(2);
        } else {
            return IMGSENSOR_RETURN_ERROR;
        }
    }
    return IMGSENSOR_RETURN_SUCCESS;
}

void Oplusimgsensor_Registdeviceinfo(char *name, char *version, kal_uint8 module_id)
{
    char *manufacture;
    if (name == NULL || version == NULL)
    {
        pr_info("name or version is NULL");
        return;
    }
    switch (module_id)
    {
        case IMGSENSOR_MODULE_ID_SUNNY:  /* Sunny */
            manufacture = DEVICE_MANUFACUTRE_SUNNY;
            break;
        case IMGSENSOR_MODULE_ID_TRULY:  /* Truly */
            manufacture = DEVICE_MANUFACUTRE_TRULY;
            break;
        case IMGSENSOR_MODULE_ID_SEMCO:  /* Semco */
            manufacture = DEVICE_MANUFACUTRE_SEMCO;
            break;
        case IMGSENSOR_MODULE_ID_LITEON:  /* Lite-ON */
            manufacture = DEVICE_MANUFACUTRE_LITEON;
            break;
        case IMGSENSOR_MODULE_ID_QTECH:  /* Q-Tech */
            manufacture = DEVICE_MANUFACUTRE_QTECH;
            break;
        case IMGSENSOR_MODULE_ID_OFILM:  /* O-Film */
            manufacture = DEVICE_MANUFACUTRE_OFILM;
            break;
        case IMGSENSOR_MODULE_ID_SHINE:  /* Shine */
            manufacture = DEVICE_MANUFACUTRE_SHINE;
            break;
        default:
            manufacture = DEVICE_MANUFACUTRE_NA;
    }
    register_device_proc(name, version, manufacture);
}

bool oplus_is_system_camera(unsigned int val)
{
    static bool is_system_cam = true;
    if(val != OPLUS_CHECK_IS_SYSTEM_CAM) {
        is_system_cam = (val != 0);
    }
    pr_debug("oplus_is_system_camera:in %d out %d\n", val, is_system_cam);

    return is_system_cam;
}

void Oplusimgsensor_powerstate_notify(bool val, enum IMGSENSOR_SENSOR_IDX sensorIdx)
{
    static int notify_cnt = 0;
    if ((is_project(21081) || is_project(0x212A1) || is_project(21851) || is_project(21876)) && (sensorIdx == IMGSENSOR_SENSOR_IDX_SUB)) {
        pr_info("Cancel Oplusimgsensor_powerstate_notify while sensorIdx[%d]!", sensorIdx);
        return;
    }

    if (val && (notify_cnt == 0)) {
        if (is_project(21081) || is_project(0x212A1) || is_project(21851) || is_project(21876)) {
            pr_info("[%s] val:%d", __func__, val);
            oplus_chg_set_camera_status(val);
            oplus_chg_set_camera_on(val);
            notify_cnt++;
            return;
        }
    }
    if (!val && (notify_cnt != 0)) {
        if (is_project(21081) || is_project(0x212A1) || is_project(21851) || is_project(21876)) {
            pr_info("[%s] val:%d", __func__, val);
            oplus_chg_set_camera_status(val);
            oplus_chg_set_camera_on(val);
            notify_cnt = 0;
            return;
        }
    }
}
