/*******************************************************************************
 *
 * Copyright (c) 2014 Bosch Software Innovations GmbH, Germany.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Bosch Software Innovations GmbH - Please refer to git log
 *
 *******************************************************************************/
/*
 * lwm2mclient.h
 *
 *  General functions of lwm2m test client.
 *
 *  Created on: 22.01.2015
 *  Author: Achim Kraus
 *  Copyright (c) 2015 Bosch Software Innovations GmbH, Germany. All rights reserved.
 */

#ifndef LWM2MCLIENT_H_
#define LWM2MCLIENT_H_

#include "liblwm2m.h"

extern int g_reboot;

#define MAX_REGS_TO_RES     6
#define MAX_DEVICE_LOG_W    500
#define MAX_DEVICE_LOG_B    1000
#define MAX_ERROR_LOG_W     60
#define MAX_ERROR_LOG_B     120
#define FRIENDLY_NAME       "alien2"
#define MODBUS_STN_ADDR     1
#define APP_TYPE_LEN        128
#define CONFIG_FILE       "/home/volume/plc_app.conf"
#define POLL_INTERVAL_CRITICAL_PRIORITY       1
#define POLL_INTERVAL_HIGH_PRIORITY           5
#define POLL_INTERVAL_MEDIUM_PRIORITY         100
#define POLL_INTERVAL_LOW_PRIORITY            17280 // decremented in a 5 second loop give approx 24 hr delay

uint16_t poll_timer_low_pri;
uint8_t modbus_stn_id;
bool operator_present;
bool operator_configured;
int poll_quit;

#define OBJ_COUNT 21
lwm2m_object_t * objArray[OBJ_COUNT];

/* create thread argument struct for poll() */
typedef struct _thread_data_t {
    int tid;
    lwm2m_object_t * objArray[OBJ_COUNT];
    lwm2m_context_t * lwm2mH;
} thread_data_t;

enum PLC_OBJECTS
{
    SECURITY_OBJ,
    SERVER_OBJ,
    DEVICE_OBJ,
    FIRMWARE_OBJ,
    LOCATION_OBJ,
    PLC_30000_0BJ,
    PLC_30001_0BJ,
    PLC_30002_0BJ,
    PLC_30003_0BJ,
    PLC_30004_0BJ,
    PLC_30005_0BJ,
    PLC_30006_0BJ,
    PLC_30007_0BJ,
    PLC_30008_0BJ,
    PLC_30009_0BJ,
    PLC_30010_0BJ,
    PLC_30011_0BJ,
    PLC_30012_0BJ,
    CONN_MON_OBJ,
    CONN_STAT_OBJ,
    ACC_CTRL_CREATE_OBJ
};

void *poll_plc(void *arg);
/*
 * object_device.c
 */
lwm2m_object_t * get_object_device(void);
void free_object_device(lwm2m_object_t * objectP);
uint8_t device_change(lwm2m_data_t * dataArray, lwm2m_object_t * objectP);
void display_device_object(lwm2m_object_t * objectP);
/*
 * object_firmware.c
 */
lwm2m_object_t * get_object_firmware(void);
void free_object_firmware(lwm2m_object_t * objectP);
void display_firmware_object(lwm2m_object_t * objectP);
/*
 * object_location.c
 */
lwm2m_object_t * get_object_location(void);
void free_object_location(lwm2m_object_t * object);
void display_location_object(lwm2m_object_t * objectP);
/*
 * plc_30000.c
 */
#define PLUNGER_LIFT_CONTROLLER 30000
lwm2m_object_t * get_30000_object(void);
void free_30000_object(lwm2m_object_t * object);
void display_30000_object(lwm2m_object_t * objectP);
uint8_t poll_30000_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
uint8_t get_modbus_address(void);
void set_modbus_address(uint8_t id);
/*
 * plc_30001.c
 */
#define PLC_STATE 30001
lwm2m_object_t * get_30001_object(void);
void free_30001_object(lwm2m_object_t * object);
void display_30001_object(lwm2m_object_t * objectP);
uint8_t poll_30001_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30002.c
 */
#define PLC_TIME_VELOCITY_SETTINGS 30002
lwm2m_object_t * get_30002_object(void);
void free_30002_object(lwm2m_object_t * object);
void display_30002_object(lwm2m_object_t * objectP);
uint8_t poll_30002_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30003.c
 */
#define PLC_OPT_SAFETY_SETTINGS 30003
lwm2m_object_t * get_30003_object(void);
void free_30003_object(lwm2m_object_t * object);
void display_30003_object(lwm2m_object_t * objectP);
uint8_t poll_30003_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30004.c
 */
#define PLC_BATTERY_STATUS 30004
lwm2m_object_t * get_30004_object(void);
void free_30004_object(lwm2m_object_t * object);
void display_30004_object(lwm2m_object_t * objectP);
uint8_t poll_30004_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30005.c
 */
#define PLC_PRESSURE_OR_FLOW_INPUT 30005
lwm2m_object_t * get_30005_object(void);
void free_30005_object(lwm2m_object_t * object);
void display_30005_object(lwm2m_object_t * objectP);
uint8_t poll_30005_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId);
/*
 * plc_30006.c
 */
#define PLC_ARRIVAL_SENSOR 30006
lwm2m_object_t * get_30006_object(void);
void free_30006_object(lwm2m_object_t * object);
void display_30006_object(lwm2m_object_t * objectP);
uint8_t poll_30006_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30007.c
 */
#define PLC_OUTPUTS 30007
lwm2m_object_t * get_30007_object(void);
void free_30007_object(lwm2m_object_t * object);
void display_30007_object(lwm2m_object_t * objectP);
uint8_t poll_30007_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30008.c
 */
#define PLC_CYCLE_LOG_RECORD 30008
lwm2m_object_t * get_30008_object(void);
void free_30008_object(lwm2m_object_t * object);
void display_30008_object(lwm2m_object_t * objectP);
uint8_t poll_30008_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId);
/*
 * plc_30009.c
 */
#define PLC_DIFF_PRESSURE_OPT 30009
lwm2m_object_t * get_30009_object(void);
void free_30009_object(lwm2m_object_t * object);
void display_30009_object(lwm2m_object_t * objectP);  
uint8_t poll_30009_object(lwm2m_object_t * obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30010.c
 */
#define PLC_FLOW_OPT 30010
lwm2m_object_t * get_30010_object(void);
void free_30010_object(lwm2m_object_t * object);
void display_30010_object(lwm2m_object_t * objectP);
uint8_t poll_30010_object(lwm2m_object_t * obj, lwm2m_context_t * lwm2mH);
/*
 * plc_30011.c
 */
#define PLC_PRODUCTION_LOG_RECORD 30011
lwm2m_object_t * get_30011_object(void);
void free_30011_object(lwm2m_object_t * object);
void display_30011_object(lwm2m_object_t * objectP);
uint8_t poll_30011_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId);
/*
 * plc_30012.c
 */
#define PLC_DEVICE_LOG 30012
lwm2m_object_t * get_30012_object(void);
void free_30012_object(lwm2m_object_t * object);
void display_30012_object(lwm2m_object_t * objectP);
uint8_t poll_30012_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId);
/*
 * object_server.c
 */
lwm2m_object_t * get_server_object(int serverId, const char* binding, int lifetime, bool storing);
void clean_server_object(lwm2m_object_t * object);
void display_server_object(lwm2m_object_t * objectP);
void copy_server_object(lwm2m_object_t * objectDest, lwm2m_object_t * objectSrc);

/*
 * object_connectivity_moni.c
 */
lwm2m_object_t * get_object_conn_m(void);
void free_object_conn_m(lwm2m_object_t * objectP);
uint8_t connectivity_moni_change(lwm2m_data_t * dataArray, lwm2m_object_t * objectP);

/*
 * object_connectivity_stat.c
 */
extern lwm2m_object_t * get_object_conn_s(void);
void free_object_conn_s(lwm2m_object_t * objectP);
extern void conn_s_updateTxStatistic(lwm2m_object_t * objectP, uint16_t txDataByte, bool smsBased);
extern void conn_s_updateRxStatistic(lwm2m_object_t * objectP, uint16_t rxDataByte, bool smsBased);

/*
 * object_access_control.c
 */
lwm2m_object_t* acc_ctrl_create_object(void);
void acl_ctrl_free_object(lwm2m_object_t * objectP);
bool  acc_ctrl_obj_add_inst (lwm2m_object_t* accCtrlObjP, uint16_t instId,
                 uint16_t acObjectId, uint16_t acObjInstId, uint16_t acOwner);
bool  acc_ctrl_oi_add_ac_val(lwm2m_object_t* accCtrlObjP, uint16_t instId,
                 uint16_t aclResId, uint16_t acValue);
/*
 * lwm2mclient.c
 */
void handle_value_changed(lwm2m_context_t* lwm2mH, lwm2m_uri_t* uri, const char * value, size_t valueLength);
/*
 * system_api.c
 */
void init_value_change(lwm2m_context_t * lwm2m);
void system_reboot(void);

/*
 * object_security.c
 */
lwm2m_object_t * get_security_object(int serverId, const char* serverUri, char * bsPskId, char * psk, uint16_t pskLen, bool isBootstrap);
void clean_security_object(lwm2m_object_t * objectP);
char * get_server_uri(lwm2m_object_t * objectP, uint16_t secObjInstID);
void display_security_object(lwm2m_object_t * objectP);
void copy_security_object(lwm2m_object_t * objectDest, lwm2m_object_t * objectSrc);

#endif /* LWM2MCLIENT_H_ */
