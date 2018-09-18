/*******************************************************************************
 *
 * Copyright (c) 2013, 2014 Intel Corporation and others.
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
 *    David Navarro, Intel Corporation - initial API and implementation
 *    domedambrosio - Please refer to git log
 *    Fabien Fleutot - Please refer to git log
 *    Axel Lorente - Please refer to git log
 *    Achim Kraus, Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *    Ville Skyttä - Please refer to git log
 *    
 *******************************************************************************/

/*
 Copyright (c) 2013, 2014 Intel Corporation

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.

 David Navarro <david.navarro@intel.com>

*/

/* September 7, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller State
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller State  | 30007 |    No     |    No     |
 *
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             | ID | Operations | Instances | Mandatory |  Type   | Range     |  Units  | Description                                   |
 *  Current Controller State         |  1 |     R      |    No     |    Yes    | Integer | 0-1       |         | 0=Valve closed, 1=Valve open                  |
 *  Open Sales Valve                 |  2 |     E      |    No     |    Yes    |         |           |         | Executes Open Sales Valve                     |
 *  Close Sales Valve                |  3 |     E      |    No     |    Yes    |         |           |         | Executes Close Sales Valve                    |
 *  Valve B Status                   |  4 |     R      |    No     |    No     | Integer | 0-1       |         | 0=Valve closed, 1=Valve open                  |
 *  Open Valve B                     |  5 |     E      |    No     |    No     |         |           |         | Executes Open Valve B                         |
 *  Close Valve B                    |  6 |     E      |    No     |    No     |         |           |         | Executes Close Valve B                        |
 *  Auto Catch Status                |  7 |     R      |    No     |    No     | Integer | 0-1       |         | 0=Valve closed, 1=Valve open                  |
 *  Open Auto Catch Valve            |  8 |     E      |    No     |    No     |         |           |         | Executes Open Auto Catch Valve                |
 *  Close Auto Catch Valve           |  9 |     E      |    No     |    No     |         |           |         | Executes Close Auto Catch Valve               |
 *  Current Controller State         | 10 |     RW     |    No     |    No     | Integer | 0-6       |         | 0=Disabled, 1=Line, Valve A only open during  |
 *                                   |    |            |           |           |         |           |         | Afterflow, 2=Line, Valve A and B open during  |
 *                                   |    |            |           |           |         |           |         | Afterflow, 3=Tank, 4=Flow Control 5=Purge     |
 *                                   |    |            |           |           |         |           |         | 6=Tank with Sales open during vent            |
 *  Valve B Purge Time               | 11 |     RW     |    No     |    No     | Integer | 1-1800000 | seconds |                                               |
 *  Valve B Tank Delay               | 12 |     RW     |    No     |    No     | Integer | 1-1799999 | seconds |                                               |
 *  Valve B Afterflow Time           | 13 |     RW     |    No     |    No     | Integer | 1-36000   | seconds |                                               |
 *  Auto Catch Config                | 14 |     RW     |    No     |    No     | Integer | 0-2       |         | 0=Disabled, 1=Enable on Rise, 2 Enable on     |
 *                                   |    |            |           |           |         |           |         | Arrival                                       |
 *  Auto Catch Hold Time             | 15 |     RW     |    No     |    No     | Integer | 1-1800000 | seconds |                                               |
 *
 */

#include "liblwm2m.h"
#include "lwm2mclient.h"
#include "modbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define PRV_TLV_BUFFER_SIZE 64
#define EPOCH_TIME_ADJ 946684800
/*
 * Multiple instance objects can use userdata to store data that will be shared between the different instances.
 * The lwm2m_object_t object structure - which represent every object of the liblwm2m as seen in the single instance
 * object - contain a chained list called instanceList with the object specific structure prv_instance_t:
 */
typedef struct _prv_instance_
{
    /*
     * The first two are mandatories and represent the pointer to the next instance and the ID of this one. The rest
     * is the instance scope user data (uint8_t test in this case)
     */
    struct _prv_instance_ * next;             // matches lwm2m_list_t::next
    uint16_t shortID;                         // matches lwm2m_list_t::id
    uint64_t salesValveStatus;                // 0-1
    uint64_t openSalesValve;                  // placeholder for executable
    uint64_t closeSalesValve;                 // placeholder for executable
    uint64_t valveBStatus;
    uint64_t openValveB;                      // placeholder for executable
    uint64_t closeValveB;                     // placeholder for executable
    uint64_t autoCatchStatus;
    uint64_t openAutoCatchValve;              // placeholder for executable
    uint64_t closeAutoCatchValve;               // placeholder for executable
    uint64_t valveBConfig;
    uint64_t valveBPurgeTime;
    uint64_t valveBTankDelay;
    uint64_t valveBAfterflowDelay;
    uint64_t autoCatchConfig;
    uint64_t autoCatchHoldTime;
} prv_instance_t;

static void prv_output_buffer(uint8_t * buffer,
                              int length)
{
    int i;
    uint8_t array[16];

    i = 0;
    while (i < length)
    {
        int j;
        fprintf(stderr, "  ");

        memcpy(array, buffer+i, 16);

        for (j = 0 ; j < 16 && i+j < length; j++)
        {
            fprintf(stderr, "%02X ", array[j]);
        }
        while (j < 16)
        {
            fprintf(stderr, "   ");
            j++;
        }
        fprintf(stderr, "  ");
        for (j = 0 ; j < 16 && i+j < length; j++)
        {
            if (isprint(array[j]))
                fprintf(stderr, "%c ", array[j]);
            else
                fprintf(stderr, ". ");
        }
        fprintf(stderr, "\n");

        i += 16;
    }
}
uint8_t poll_30007_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];   

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, 0);
    /* This part is needed to trigger the watcher of the observable */
    if (lwm2m_stringToUri("/30007/0/1", (sizeof("/30007/0/1") - 1), &uri))
    {
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 21, 1, char_buf);
        targetP->salesValveStatus = char_buf[0]; 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "salesValveStatus: %d\n", targetP->salesValveStatus);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/4", (sizeof("/30007/0/4") - 1), &uri))
    {
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 22, 1, char_buf);
        targetP->valveBStatus = char_buf[0]; 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "valveBStatus: %d\n", targetP->valveBStatus);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/7", (sizeof("/30007/0/7") - 1), &uri))
    {
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 23, 1, char_buf);
        targetP->autoCatchStatus = char_buf[0]; 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "autoCatchStatus: %d\n", targetP->autoCatchStatus);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/10", (sizeof("/30007/0/10") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 205, 1, reg_buf);
        targetP->valveBConfig = reg_buf[0]; 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "valveBConfig: %d\n", targetP->valveBConfig);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/11", (sizeof("/30007/0/11") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 152, 2, reg_buf);
        targetP->valveBPurgeTime = reg_buf[0];
        targetP->valveBPurgeTime <<= 8;
        targetP->valveBPurgeTime += reg_buf[1]; 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "valveBPurgeTime: %d\n", targetP->valveBPurgeTime);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/12", (sizeof("/30007/0/12") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 110, 2, reg_buf);
        targetP->valveBTankDelay = reg_buf[0];
        targetP->valveBTankDelay <<= 8;
        targetP->valveBTankDelay += reg_buf[1];
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "valveBTankDelay: %d\n", targetP->valveBTankDelay);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/13", (sizeof("/30007/0/13") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 119, 2, reg_buf);
        targetP->valveBAfterflowDelay = reg_buf[0];
        targetP->valveBAfterflowDelay <<= 8;
        targetP->valveBAfterflowDelay += reg_buf[1];
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "valveBAfterflowDelay: %d\n", targetP->valveBAfterflowDelay);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/14", (sizeof("/30007/0/14") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 208, 1, reg_buf);
        targetP->autoCatchConfig = reg_buf[0];
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "autoCatchConfig: %d\n", targetP->autoCatchConfig);
#endif    
    }
    if (lwm2m_stringToUri("/30007/0/15", (sizeof("/30007/0/15") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 155, 2, reg_buf);
        targetP->autoCatchHoldTime = reg_buf[0];
        targetP->autoCatchHoldTime <<= 8;
        targetP->autoCatchHoldTime += reg_buf[1];
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "autoCatchHoldTime: %d\n", targetP->autoCatchHoldTime);
#endif    
    }
}
static uint8_t prv_read(uint16_t instanceId,
                        int * numDataP,
                        lwm2m_data_t ** dataArrayP,
                        lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];

    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(9);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 9;
        (*dataArrayP)[0].id = 1;
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 21, 1, char_buf);
        targetP->salesValveStatus = char_buf[0];
        lwm2m_data_encode_int(targetP->salesValveStatus, *dataArrayP + 0);
        (*dataArrayP)[1].id = 4;
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 22, 1, char_buf);
        targetP->valveBStatus = char_buf[0];
        lwm2m_data_encode_int(targetP->valveBStatus, *dataArrayP + 1);
        (*dataArrayP)[2].id = 7;
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 23, 1, char_buf);
        targetP->autoCatchStatus = char_buf[0];
        lwm2m_data_encode_int(targetP->autoCatchStatus, *dataArrayP + 2);
        (*dataArrayP)[3].id = 10;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 205, 1, reg_buf);
        targetP->valveBConfig = reg_buf[0];
        lwm2m_data_encode_int(targetP->valveBConfig, *dataArrayP + 3);
        (*dataArrayP)[4].id = 11;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 152, 2, reg_buf);
        targetP->valveBPurgeTime = reg_buf[0];
        targetP->valveBPurgeTime <<= 8;
        targetP->valveBPurgeTime += reg_buf[1];
        lwm2m_data_encode_int(targetP->valveBPurgeTime, *dataArrayP + 4); 
        (*dataArrayP)[5].id = 12;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 110, 2, reg_buf);
        targetP->valveBTankDelay = reg_buf[0];
        targetP->valveBTankDelay <<= 8;
        targetP->valveBTankDelay += reg_buf[1];
        lwm2m_data_encode_int(targetP->valveBTankDelay, *dataArrayP + 5); 
        (*dataArrayP)[6].id = 13;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 119, 2, reg_buf);
        targetP->valveBAfterflowDelay = reg_buf[0];
        targetP->valveBAfterflowDelay <<= 8;
        targetP->valveBAfterflowDelay += reg_buf[1];
        lwm2m_data_encode_int(targetP->valveBAfterflowDelay, *dataArrayP + 6);  
        (*dataArrayP)[7].id = 14;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 208, 1, reg_buf);
        targetP->autoCatchConfig = reg_buf[0];
        lwm2m_data_encode_int(targetP->autoCatchConfig, *dataArrayP + 7);        
        (*dataArrayP)[8].id = 15;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 155, 2, reg_buf);
        targetP->autoCatchHoldTime = reg_buf[0];
        targetP->autoCatchHoldTime <<= 8;
        targetP->autoCatchHoldTime += reg_buf[1];
        lwm2m_data_encode_int(targetP->autoCatchHoldTime, *dataArrayP + 8);
    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 21, 1, char_buf);
                targetP->salesValveStatus = char_buf[0];
                lwm2m_data_encode_int(targetP->salesValveStatus, *dataArrayP + i);
                break;
            case 2:
            case 3:
                return COAP_405_METHOD_NOT_ALLOWED;
                break;
            case 4:
                result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 22, 1, char_buf);
                targetP->valveBStatus = char_buf[0];
                lwm2m_data_encode_int(targetP->valveBStatus, *dataArrayP + i);
                break;
            case 5:
            case 6:
                return COAP_405_METHOD_NOT_ALLOWED;
                break;
            case 7:
                result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 23, 1, char_buf);
                targetP->autoCatchStatus = char_buf[0];
                lwm2m_data_encode_int(targetP->autoCatchStatus, *dataArrayP + i);
                break;
            case 8:
            case 9:
                return COAP_405_METHOD_NOT_ALLOWED;
                break;
            case 10:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 205, 1, reg_buf);
                targetP->valveBConfig = reg_buf[0];
                lwm2m_data_encode_int(targetP->valveBConfig, *dataArrayP + i);
                break;
            case 11:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 152, 2, reg_buf);
                targetP->valveBPurgeTime = reg_buf[0];
                targetP->valveBPurgeTime <<= 8;
                targetP->valveBPurgeTime += reg_buf[1];
                lwm2m_data_encode_int(targetP->valveBPurgeTime, *dataArrayP + i);
                break;
            case 12:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 110, 2, reg_buf);
                targetP->valveBTankDelay = reg_buf[0];
                targetP->valveBTankDelay <<= 8;
                targetP->valveBTankDelay += reg_buf[1];
                lwm2m_data_encode_int(targetP->valveBTankDelay, *dataArrayP + i);
                break;
            case 13:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 119, 2, reg_buf);
                targetP->valveBAfterflowDelay = reg_buf[0];
                targetP->valveBAfterflowDelay <<= 8;
                targetP->valveBAfterflowDelay += reg_buf[1];
                lwm2m_data_encode_int(targetP->valveBAfterflowDelay, *dataArrayP + i);
                break;
            case 14:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 208, 1, reg_buf);
                targetP->autoCatchConfig = reg_buf[0];
                lwm2m_data_encode_int(targetP->autoCatchConfig, *dataArrayP + i);  
                break;
            case 15:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 155, 2, reg_buf);
                targetP->autoCatchHoldTime = reg_buf[0];
                targetP->autoCatchHoldTime <<= 8;
                targetP->autoCatchHoldTime += reg_buf[1];
                lwm2m_data_encode_int(targetP->autoCatchHoldTime, *dataArrayP + i);
                break;
            default:
                return COAP_404_NOT_FOUND;
            }
        }
    }
    return COAP_205_CONTENT;
}

static uint8_t prv_discover(uint16_t instanceId,
                            int * numDataP,
                            lwm2m_data_t ** dataArrayP,
                            lwm2m_object_t * objectP)
{
    int i;

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(15);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 15;
        (*dataArrayP)[0].id = 1;
        (*dataArrayP)[1].id = 2;
        (*dataArrayP)[2].id = 3;
        (*dataArrayP)[3].id = 4;
        (*dataArrayP)[4].id = 5;
        (*dataArrayP)[5].id = 6;
        (*dataArrayP)[6].id = 7;
        (*dataArrayP)[7].id = 8;
        (*dataArrayP)[8].id = 9;
        (*dataArrayP)[9].id = 10;
        (*dataArrayP)[10].id = 11;
        (*dataArrayP)[11].id = 12;
        (*dataArrayP)[12].id = 13;
        (*dataArrayP)[13].id = 14;
        (*dataArrayP)[14].id = 15;
    }
    else
    {
        for (i = 0; i < *numDataP; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:                
                break;
            default:
                return COAP_404_NOT_FOUND;
            }
        }
    }

    return COAP_205_CONTENT;
}

static uint8_t prv_write(uint16_t instanceId,
                         int numData,
                         lwm2m_data_t * dataArray,
                         lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            return COAP_405_METHOD_NOT_ALLOWED;
            break;
        case 10:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->valveBConfig)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->valveBConfig;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 205, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break; 
        case 11:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->valveBPurgeTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->valveBPurgeTime >> 16;
            reg_buf[1] = targetP->valveBPurgeTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 152, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 12:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->valveBTankDelay)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->valveBTankDelay >> 16;
            reg_buf[1] = targetP->valveBTankDelay;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 110, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 13:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->valveBAfterflowDelay)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->valveBAfterflowDelay >> 16;
            reg_buf[1] = targetP->valveBAfterflowDelay;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 119, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 14:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->autoCatchConfig)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->autoCatchConfig;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 208, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 15:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->autoCatchHoldTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->autoCatchHoldTime >> 16;
            reg_buf[1] = targetP->autoCatchHoldTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 155, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        default:
            return COAP_404_NOT_FOUND;
        }
    }
    return COAP_204_CHANGED;
}

static uint8_t prv_delete(uint16_t id,
                          lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;

    objectP->instanceList = lwm2m_list_remove(objectP->instanceList, id, (lwm2m_list_t **)&targetP);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    lwm2m_free(targetP);

    return COAP_202_DELETED;
}

static uint8_t prv_create(uint16_t instanceId,
                          int numData,
                          lwm2m_data_t * dataArray,
                          lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    uint8_t result;


    targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
    if (NULL == targetP) return COAP_500_INTERNAL_SERVER_ERROR;
    memset(targetP, 0, sizeof(prv_instance_t));

    targetP->shortID = instanceId;
    objectP->instanceList = LWM2M_LIST_ADD(objectP->instanceList, targetP);

    result = prv_write(instanceId, numData, dataArray, objectP);

    if (result != COAP_204_CHANGED)
    {
        (void)prv_delete(instanceId, objectP);
    }
    else
    {
        result = COAP_201_CREATED;
    }

    return result;
}

static uint8_t prv_exec(uint16_t instanceId,
                        uint16_t resourceId,
                        uint8_t * buffer,
                        int length,
                        lwm2m_object_t * objectP)
{
    int result;
    unsigned char buf[] = { 1, 0 };
    
    if (NULL == lwm2m_list_find(objectP->instanceList, instanceId)) return COAP_404_NOT_FOUND;

    switch (resourceId)
    {
    case 1:
        return COAP_405_METHOD_NOT_ALLOWED;
    case 2:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 13, buf, 1);
        return COAP_204_CHANGED;
    case 3:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 14, buf, 1);
        return COAP_204_CHANGED;
    case 4:
        return COAP_405_METHOD_NOT_ALLOWED;
    case 5:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 15, buf, 1);
        return COAP_204_CHANGED;
    case 6:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 16, buf, 1);
        return COAP_204_CHANGED;
    case 7:
        return COAP_405_METHOD_NOT_ALLOWED;
    case 8:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 17, buf, 1);
        return COAP_204_CHANGED;
    case 9:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 18, buf, 1);
        return COAP_204_CHANGED;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30007_object(lwm2m_object_t * object)
{
#ifdef WITH_LOGS
    fprintf(stdout, "  /%u: Test object, instances:\r\n", object->objID);
    prv_instance_t * instance = (prv_instance_t *)object->instanceList;
    while (instance != NULL)
    {
        //fprintf(stdout, "    /%u/%u: shortId: %u, test: %u\r\n",
                //object->objID, instance->shortID,
                //instance->shortID, instance->test);
        fprintf(stdout, "    /%u/%u:", object->objID, instance->shortID);
        instance = (prv_instance_t *)instance->next;
    }
#endif
}
uint64_t salesValveStatus;                 // 0-1
uint64_t openSalesValve;                   // placeholder for executable
uint64_t closeSalesValve;                  // placeholder for executable
uint64_t valveBStatus;
uint64_t openValveB;                       // placeholder for executable
uint64_t closeValveB;                      // placeholder for executable
uint64_t autoCatchStatus;
uint64_t openAutoCatchValve;               // placeholder for executable
uint64_t closeAutoCatchValve;                // placeholder for executable
uint64_t valveBConfig;
uint64_t valveBPurgeTime;
uint64_t valveBTankDelay;
uint64_t valveBAfterflowDelay;
uint64_t autoCatchConfig;
uint64_t autoCatchHoldTime;
lwm2m_object_t * get_30007_object(void)
{
    lwm2m_object_t * obj30007;

    obj30007 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30007)
    {
        int i;
        prv_instance_t * targetP;
        obj30007->objID = PLC_OUTPUTS;
        targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
        if (NULL == targetP) return NULL;
        memset(targetP, 0, sizeof(prv_instance_t));
        targetP->shortID                    = 0;
        targetP->salesValveStatus           = 0;
        targetP->openSalesValve             = 0;
        targetP->closeSalesValve            = 0;
        targetP->valveBStatus               = false;
        targetP->openValveB                 = 0;
        targetP->closeValveB                = 0;
        targetP->autoCatchStatus            = 0;
        targetP->openAutoCatchValve         = 0;
        targetP->closeAutoCatchValve        = 0;
        targetP->valveBConfig               = 0;
        targetP->valveBPurgeTime            = 0;
        targetP->valveBTankDelay            = 0;
        targetP->valveBAfterflowDelay       = 0;
        targetP->autoCatchConfig            = 0;
        targetP->autoCatchHoldTime          = 0;
        obj30007->instanceList = LWM2M_LIST_ADD(obj30007->instanceList, targetP);
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30007->readFunc = prv_read;
        obj30007->discoverFunc = prv_discover;
        obj30007->writeFunc = prv_write;
        obj30007->executeFunc = prv_exec;
        obj30007->createFunc = prv_create;
        obj30007->deleteFunc = prv_delete;
        obj30007->userData = NULL;   // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30007;
}

void free_30007_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

