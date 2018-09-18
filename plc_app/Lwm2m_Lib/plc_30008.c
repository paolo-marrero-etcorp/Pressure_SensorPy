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

/* September 12, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller Plunger Cycle Log Record
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller        | 30008 |    Yes    |    No     |
 *  Time and Velocity Settings
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             | ID | Operations | Instances | Mandatory |  Type   | Range |  Units         |           Description                                     |
 *  Cycle Start Time                 |  1 |     R      |    No     |    Yes    | Time    |       |                | App must convert time datum                               |
 *  Cycle Log Type                   |  2 |     R      |    No     |    No     | Integer | 0-8   |                | 0=Normal, 1=Fast Trip, 2=Non-Arrival, 3=Max Open          |
 *                                   |    |            |           |           |         |       |                | 4=Low Battery, 5=Operator Change, 6=Line Pressure Shut In | 
 *                                   |    |            |           |           |         |       |                | 7=Start-up, 8=Danger Velocity                             |
 *  Cycle Rise Time                  |  3 |     R      |    No     |    No     | Integer |       | seconds        |                                                           |
 *  Cycle Afterflow Time             |  4 |     R      |    No     |    No     | Integer |       | seconds        |                                                           |
 *  Cycle Close Time                 |  5 |     R      |    No     |    No     | Integer |       | seconds        |                                                           |
 *  Cycle Vent Time                  |  6 |     R      |    No     |    No     | Integer |       | seconds        |                                                           |
 *  Cycle Average Velocity           |  7 |     R      |    No     |    No     | Float   |       | Units Resource |                                                           |
 *  Cycle Surface Velocity           |  8 |     R      |    No     |    No     | Float   |       | Units Resource |                                                           |
 *  Cycle Surface Velocity Code      |  9 |     R      |    No     |    No     | Integer |       |                | 0, 1, 22, 23 = Velocity Calc Error, (2-8)= Velocity Valid |                                                         |
 *                                   |    |            |           |           |         |       |                | 20=Velocity Over-range                                    |
 *  Units                            | 5701 |   R      |    No     |    No     | String  |       | m/min or ft/min                                                            |
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
    struct _prv_instance_ * next;   // matches lwm2m_list_t::next
    uint16_t    shortID;               // matches lwm2m_list_t::id
    uint64_t    cycleStartTime;
    uint64_t    cycleLogType;
    uint64_t    cycleRiseTime;
    uint64_t    cycleAfterflowTime;
    uint64_t    cycleCloseTime;
    uint64_t    cycleVentTime;
    double      cycleAverageVelocity;
    double      cycleSurfaceVelocity;
    uint64_t    cycleSurfaceVelocityCode;
    char        sensorUnits[7];           // Resource 5701: Read from 0:0010, send "ft/min" = 0 or "m/min" = 1 accordingly
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
uint8_t poll_30008_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i;
    int result;
    int modbusReg;
    char char_uri[32];
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];   

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, instanceId);
    /* This part is needed to trigger the watcher of the observable */
    sprintf(char_uri, "/30008/%d/1", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6002 + (6 * instanceId);
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->cycleStartTime = reg_buf[0];
            targetP->cycleStartTime <<= 16;
            targetP->cycleStartTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleStartTime: %d\n", targetP->cycleStartTime);
#endif    
    }
    sprintf(char_uri, "/30008/%d/2", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6152 + instanceId;
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->cycleLogType = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleLogType: %d\n", targetP->cycleLogType);
#endif    
    }
    sprintf(char_uri, "/30008/%d/3", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6177 + (3 * instanceId);
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->cycleRiseTime = reg_buf[0];
            targetP->cycleRiseTime <<= 16;
            targetP->cycleRiseTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleRiseTime: %d\n", targetP->cycleRiseTime);
#endif    
    }
    sprintf(char_uri, "/30008/%d/4", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6252 + (3 * instanceId);
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->cycleAfterflowTime = reg_buf[0];
            targetP->cycleAfterflowTime <<= 16;
            targetP->cycleAfterflowTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleAfterflowTime: %d\n", targetP->cycleAfterflowTime);
#endif    
    }
    sprintf(char_uri, "/30008/%d/5", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6327 + (3 * instanceId);
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->cycleCloseTime = reg_buf[0];
            targetP->cycleCloseTime <<= 16;
            targetP->cycleCloseTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleCloseTime: %d\n", targetP->cycleCloseTime);
#endif    
    }
    sprintf(char_uri, "/30008/%d/6", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6402 + (3 * instanceId);
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->cycleVentTime = reg_buf[0];
            targetP->cycleVentTime <<= 16;
            targetP->cycleVentTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleVentTime: %d\n", targetP->cycleVentTime);
#endif    
    }
    sprintf(char_uri, "/30008/%d/7", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6477 + instanceId;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->cycleAverageVelocity = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleAverageVelocity: %f\n", targetP->cycleAverageVelocity);
#endif    
    }
    sprintf(char_uri, "/30008/%d/8", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6502 + instanceId;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->cycleSurfaceVelocity = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleSurfaceVelocity: %f\n", targetP->cycleSurfaceVelocity);
#endif    
    }
    sprintf(char_uri, "/30008/%d/9", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        modbusReg = 6527 + instanceId;
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->cycleSurfaceVelocityCode = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "cycleSurfaceVelocityCode: %d\n", targetP->cycleSurfaceVelocityCode);
#endif    
    }
    sprintf(char_uri, "/30008/%d/5701", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->sensorUnits, 0x00, 7);
            if (char_buf[0] == 0x00)
            {
                strcpy(targetP->sensorUnits, "ft/min");
            }
            else
            {
                strcpy(targetP->sensorUnits, "m/min");
            }                        
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "sensorUnits: %s\n", targetP->sensorUnits);
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
    int modbusReg;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(10);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 10;
        (*dataArrayP)[0].id = 1;
        lwm2m_data_encode_int((targetP->cycleStartTime + EPOCH_TIME_ADJ), *dataArrayP + 0);
        (*dataArrayP)[1].id = 2;
        lwm2m_data_encode_int(targetP->cycleLogType, *dataArrayP + 1);
        (*dataArrayP)[2].id = 3;
        lwm2m_data_encode_int(targetP->cycleRiseTime, *dataArrayP + 2);
        (*dataArrayP)[3].id = 4;
        lwm2m_data_encode_int(targetP->cycleAfterflowTime, *dataArrayP + 3);
        (*dataArrayP)[4].id = 5;
        lwm2m_data_encode_int(targetP->cycleCloseTime, *dataArrayP + 4);
        (*dataArrayP)[5].id = 6;
        lwm2m_data_encode_int(targetP->cycleVentTime, *dataArrayP + 5);
        (*dataArrayP)[6].id = 7;
        lwm2m_data_encode_float(targetP->cycleAverageVelocity, *dataArrayP + 6);
        (*dataArrayP)[7].id = 8;
        lwm2m_data_encode_float(targetP->cycleSurfaceVelocity, *dataArrayP + 7);
        (*dataArrayP)[8].id = 9;
        lwm2m_data_encode_int(targetP->cycleSurfaceVelocityCode, *dataArrayP + 8);
        (*dataArrayP)[9].id = 5701; // resource 5701
        lwm2m_data_encode_string(targetP->sensorUnits, *dataArrayP + 9);
    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                modbusReg = 6002 + (6 * instanceId);
                result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->cycleStartTime = reg_buf[0];
                    targetP->cycleStartTime <<= 16;
                    targetP->cycleStartTime += reg_buf[1];
                    lwm2m_data_encode_int((targetP->cycleStartTime + EPOCH_TIME_ADJ), *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 2:
                modbusReg = 6152 + instanceId;
                result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->cycleLogType = reg_buf[0];
                    lwm2m_data_encode_int(targetP->cycleLogType, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 3:
                modbusReg = 6177 + (3 * instanceId);
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->cycleRiseTime = reg_buf[0];
                    targetP->cycleRiseTime <<= 16;
                    targetP->cycleRiseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->cycleRiseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 4:
                modbusReg = 6252 + (3 * instanceId);
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->cycleAfterflowTime = reg_buf[0];
                    targetP->cycleAfterflowTime <<= 16;
                    targetP->cycleAfterflowTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->cycleAfterflowTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5:
                modbusReg = 6327 + (3 * instanceId);
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->cycleCloseTime = reg_buf[0];
                    targetP->cycleCloseTime <<= 16;
                    targetP->cycleCloseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->cycleCloseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 6:
                modbusReg = 6402 + (3 * instanceId);
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->cycleVentTime = reg_buf[0];
                    targetP->cycleVentTime <<= 16;
                    targetP->cycleVentTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->cycleVentTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 7:
                modbusReg = 6477 + instanceId;
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->cycleAverageVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->cycleAverageVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 8:
                modbusReg = 6502 + instanceId;
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->cycleSurfaceVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->cycleSurfaceVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 9:
                modbusReg = 6527 + instanceId;
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->cycleSurfaceVelocityCode = reg_buf[0];
                    lwm2m_data_encode_int(targetP->cycleSurfaceVelocityCode, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5701:
                result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
                if (result == 0)
                {
                    memset(targetP->sensorUnits, 0x00, 7);
                    if (char_buf[0] == 0x00)
                    {
                        strcpy(targetP->sensorUnits, "ft/min");
                    }
                    else
                    {
                        strcpy(targetP->sensorUnits, "m/min");
                    }                        
                    lwm2m_data_encode_string(targetP->sensorUnits, *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
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
        *dataArrayP = lwm2m_data_new(10);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 10;
        (*dataArrayP)[0].id = 1;
        (*dataArrayP)[1].id = 2;
        (*dataArrayP)[2].id = 3;
        (*dataArrayP)[3].id = 4;
        (*dataArrayP)[4].id = 5;
        (*dataArrayP)[5].id = 6;
        (*dataArrayP)[6].id = 7;
        (*dataArrayP)[7].id = 8;
        (*dataArrayP)[8].id = 9;
        (*dataArrayP)[9].id = 5701;
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
            case 5701:
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
        case 5701:  // res == 5701
            return COAP_405_METHOD_NOT_ALLOWED;
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
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 5701:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30008_object(lwm2m_object_t * object)
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

lwm2m_object_t * get_30008_object(void)
{
    lwm2m_object_t * obj30008;

    obj30008 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30008)
    {
        int i;
        prv_instance_t * targetP;

        memset(obj30008, 0, sizeof(lwm2m_object_t));

        obj30008->objID = PLC_CYCLE_LOG_RECORD;
        for (i = 0; i < 25; i++)
        {
            targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
            if (NULL == targetP) return NULL;
            memset(targetP, 0, sizeof(prv_instance_t));
            targetP->shortID                       = i;
            targetP->cycleStartTime                = 0;
            targetP->cycleLogType                  = 0;
            targetP->cycleRiseTime                 = 0;
            targetP->cycleAfterflowTime            = 0;
            targetP->cycleCloseTime                = 0;
            targetP->cycleVentTime                 = 0;
            targetP->cycleAverageVelocity          = 0;
            targetP->cycleSurfaceVelocity          = 0;
            targetP->cycleSurfaceVelocityCode      = 0;
            memset(targetP->sensorUnits, 0x00, 7);
            obj30008->instanceList = LWM2M_LIST_ADD(obj30008->instanceList, targetP);
        }
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30008->readFunc = prv_read;
        obj30008->discoverFunc = prv_discover;
        obj30008->writeFunc = prv_write;
        obj30008->executeFunc = prv_exec;
        obj30008->createFunc = prv_create;
        obj30008->deleteFunc = prv_delete;
        obj30008->userData = NULL;   // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30008;
}

void free_30008_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

