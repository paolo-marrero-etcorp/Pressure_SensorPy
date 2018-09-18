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

/* September 18, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller Device Log
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller        | 30012 |    Yes    |    No     |
 *  Time and Velocity Settings
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             |  ID  | Operations | Instances | Mandatory |  Type   | Range   |  Units             |           Description                                     |
 *  Log Type                         |   1  |     RW     |    No     |    No     | Integer | 0-10    |                    | 0=Disabled, 3=Line Pressure, 4=Tubing Pressure            |
 *                                   |      |            |           |           |         |         |                    | 5=Casing Pressure, 6=Line Differential Pressure 9=Casing  | 
 *                                   |      |            |           |           |         |         |                    | Line Pressure, 10=Flow Rate, Unused values are reserved   |
 *  Log Frequency                    |   2  |     RW     |    No     |    No     | Integer | 0-36000 | seconds            |                                                           |
 *  Last Sample Time                 |   3  |     RW     |    No     |    No     | Time    |         |                    | UTC, App must convert time datum                          |
 *  LogData                          | 4014 |     R      |    No     |    Yes    | Opaque  |         | see Units Resource | Read access on that Resource returns the data collection  |
 *                                   |      |            |           |           |         |         |                    | associated to the current object instance                 |
 *  Sensor Units                     | 5701 |     R      |    No     |    No     | String  |         |                    | String depending on the units setting of the controller   |
 *                                   |      |            |           |           |         |         |                    | and type of log being retreived                           |
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
    struct _prv_instance_ * next;          // matches lwm2m_list_t::next
    uint16_t    shortID;                   // matches lwm2m_list_t::id
    uint64_t    logType;
    uint64_t    logFrequency;
    uint64_t    lastSampleTime;
    uint8_t     logData[MAX_DEVICE_LOG_B];             // max 500 16 bit samples = 1000 bytes
    char        sensorUnits[7];           // Resource 5701: Read from 0:0010, send Imperial = 0 or metric = 1 accordingly
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
uint8_t poll_30012_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i, n, j;
    int result;
    int modbusReg;
    int numModbusRegs;
    char char_uri[32];
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];  
    unsigned short log_buf[MAX_DEVICE_LOG_W];

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, instanceId);
    /* This part is needed to trigger the watcher of the observable */
    sprintf(char_uri, "/30012/%d/1", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 401;    
        }
        else
        {
            modbusReg = 405;
        }
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logType = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logType: %d\n", targetP->logType);
#endif    
    }
    sprintf(char_uri, "/30012/%d/2", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 402;    
        }
        else
        {
            modbusReg = 406;
        }
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logFrequency = reg_buf[0];
            targetP->logFrequency <<= 16;
            targetP->logFrequency += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logFrequency: %d\n", targetP->logFrequency);
#endif    
    }
    sprintf(char_uri, "/30012/%d/3", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 2102;    
        }
        else
        {
            modbusReg = 3002;
        }
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->lastSampleTime = reg_buf[0];
            targetP->lastSampleTime <<= 16;
            targetP->lastSampleTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "lastSampleTime: %d\n", targetP->lastSampleTime);
#endif    
    }
    sprintf(char_uri, "/30012/%d/4014", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 2108;
            result = read_input(FRIENDLY_NAME, modbus_stn_id, 2101, 1, reg_buf);
            if ((result == 0) && (reg_buf[0] != 0))
            {
                numModbusRegs = reg_buf[0];
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }
        }
        else
        {
            modbusReg = 3008;
            result = read_input(FRIENDLY_NAME, modbus_stn_id, 3001, 1, reg_buf);
            if ((result == 0) && (reg_buf[0] != 0))
            {
                numModbusRegs = reg_buf[0];
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, numModbusRegs, log_buf);
        if (result == 0)
        {
            j = 0;
            for (n = 0; n < numModbusRegs; n++)
            {
                targetP->logData[j] = log_buf[i] >> 8;
                j++;
                targetP->logData[j] = log_buf[i];
                j++;
            }
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "numModbusRegs: %d\n", numModbusRegs);
#endif    
    }
    sprintf(char_uri, "/30012/%d/5701", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 401;    
        }
        else
        {
            modbusReg = 405;
        }            
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->sensorUnits, 0x00, 7);
            if (char_buf[0] == 0x00)
            {
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    switch (reg_buf[0])
                    {
                    case 0:
                        memset(targetP->sensorUnits, 0x00, 7);
                        break;
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 9:
                        strcpy(targetP->sensorUnits, "PSI");
                        break;
                    case 10:
                        strcpy(targetP->sensorUnits, "ft/min");
                        break;
                    }
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
            }
            else
            {
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    switch (reg_buf[0])
                    {
                    case 0:
                        memset(targetP->sensorUnits, 0x00, 7);
                        break;
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 9:
                        strcpy(targetP->sensorUnits, "kPa");
                        break;
                    case 10:
                        strcpy(targetP->sensorUnits, "m/min");
                        break;
                    }
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
            }                        
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "sensorUnits: %d\n", targetP->sensorUnits);
#endif    
    }
}
static uint8_t prv_read(uint16_t instanceId,
                        int * numDataP,
                        lwm2m_data_t ** dataArrayP,
                        lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i, j, n;
    int result;
    int modbusReg;
    int numModbusRegs;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned short log_buf[MAX_DEVICE_LOG_W];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(5);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 5;
        (*dataArrayP)[0].id = 1;
        if (instanceId == 0)
        {
            modbusReg = 401;    
        }
        else
        {
            modbusReg = 405;
        }
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logType = reg_buf[0];
            lwm2m_data_encode_int(targetP->logType, *dataArrayP + 0); 
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }
        (*dataArrayP)[1].id = 2;
        if (instanceId == 0)
        {
            modbusReg = 402;    
        }
        else
        {
            modbusReg = 406;
        }
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logFrequency = reg_buf[0];
            targetP->logFrequency <<= 16;
            targetP->logFrequency += reg_buf[1];
            lwm2m_data_encode_int(targetP->logFrequency, *dataArrayP + 1);     
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }
        (*dataArrayP)[2].id = 3;
        if (instanceId == 0)
        {
            modbusReg = 2102;    
        }
        else
        {
            modbusReg = 3002;
        }
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->lastSampleTime = reg_buf[0];
            targetP->lastSampleTime <<= 16;
            targetP->lastSampleTime += reg_buf[1];
            lwm2m_data_encode_int((targetP->lastSampleTime + EPOCH_TIME_ADJ), *dataArrayP + 2);   
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }
        (*dataArrayP)[3].id = 4014;
        if (instanceId == 0)
        {
            modbusReg = 2108;
            result = read_input(FRIENDLY_NAME, modbus_stn_id, 2101, 1, reg_buf);
            if ((result == 0) && (reg_buf[0] != 0))
            {
                numModbusRegs = reg_buf[0];
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }
        }
        else
        {
            modbusReg = 3008;
            result = read_input(FRIENDLY_NAME, modbus_stn_id, 3001, 1, reg_buf);
            if ((result == 0) && (reg_buf[0] != 0))
            {
                numModbusRegs = reg_buf[0];
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, numModbusRegs, log_buf);
        if (result == 0)
        {
            j = 0;
            for (n = 0; n < numModbusRegs; n++)
            {
                targetP->logData[j] = log_buf[i] >> 8;
                j++;
                targetP->logData[j] = log_buf[i];
                j++;
            }
            lwm2m_data_encode_opaque(targetP->logData, j, *dataArrayP + 3);
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                
        (*dataArrayP)[4].id = 5701; // resource 5701
        if(instanceId == 0)
        {
            modbusReg = 401;    
        }
        else
        {
            modbusReg = 405;
        }
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->sensorUnits, 0x00, 7);
            if (char_buf[0] == 0x00)
            {
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    switch (reg_buf[0])
                    {
                    case 0:
                        memset(targetP->sensorUnits, 0x00, 7);
                        break;
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 9:
                        strcpy(targetP->sensorUnits, "PSI");
                        break;
                    case 10:
                        strcpy(targetP->sensorUnits, "ft/min");
                        break;
                    }
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
            }
            else
            {
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    switch (reg_buf[0])
                    {
                    case 0:
                        memset(targetP->sensorUnits, 0x00, 7);
                        break;
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 9:
                        strcpy(targetP->sensorUnits, "kPa");
                        break;
                    case 10:
                        strcpy(targetP->sensorUnits, "m/min");
                        break;
                    }
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
            }                        
            lwm2m_data_encode_string(targetP->sensorUnits, *dataArrayP + 4);
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                

    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                if (instanceId == 0)
                {
                    modbusReg = 401;    
                }
                else
                {
                    modbusReg = 405;
                }
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->logType = reg_buf[0];
                    lwm2m_data_encode_int(targetP->logType, *dataArrayP + i); 
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 2:
                if (instanceId == 0)
                {
                    modbusReg = 402;    
                }
                else
                {
                    modbusReg = 406;
                }
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->logFrequency = reg_buf[0];
                    targetP->logFrequency <<= 16;
                    targetP->logFrequency += reg_buf[1];
                    lwm2m_data_encode_int(targetP->logFrequency, *dataArrayP + i);     
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 3:
                if (instanceId == 0)
                {
                    modbusReg = 2102;    
                }
                else
                {
                    modbusReg = 3002;
                }
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
                if (result == 0)
                {
                    targetP->lastSampleTime = reg_buf[0];
                    targetP->lastSampleTime <<= 16;
                    targetP->lastSampleTime += reg_buf[1];
                    lwm2m_data_encode_int((targetP->lastSampleTime + EPOCH_TIME_ADJ), *dataArrayP + i);   
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 4014:
                if (instanceId == 0)
                {
                    modbusReg = 2108;
                    result = read_input(FRIENDLY_NAME, modbus_stn_id, 2101, 1, reg_buf);
                    if ((result == 0) && (reg_buf[0] != 0))
                    {
                        numModbusRegs = reg_buf[0];
                    }
                    else
                    {
                        return COAP_400_BAD_REQUEST;
                    }
                }
                else
                {
                    modbusReg = 3008;
                    result = read_input(FRIENDLY_NAME, modbus_stn_id, 3001, 1, reg_buf);
                    if ((result == 0) && (reg_buf[0] != 0))
                    {
                        numModbusRegs = reg_buf[0];
                    }
                    else
                    {
                        return COAP_400_BAD_REQUEST;
                    }
                }
                result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, numModbusRegs, log_buf);
                if (result == 0)
                {
                    j = 0;
                    for (n = 0; n < numModbusRegs; n++)
                    {
                        targetP->logData[j] = log_buf[i] >> 8;
                        j++;
                        targetP->logData[j] = log_buf[i];
                        j++;
                    }
                    lwm2m_data_encode_opaque(targetP->logData, j, *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5701:
                if (instanceId == 0)
                {
                    modbusReg = 401;    
                }
                else
                {
                    modbusReg = 405;
                }            
                result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
                if (result == 0)
                {
                    memset(targetP->sensorUnits, 0x00, 7);
                    if (char_buf[0] == 0x00)
                    {
                        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                        if (result == 0)
                        {
                            switch (reg_buf[0])
                            {
                            case 0:
                                memset(targetP->sensorUnits, 0x00, 7);
                                break;
                            case 3:
                            case 4:
                            case 5:
                            case 6:
                            case 9:
                                strcpy(targetP->sensorUnits, "PSI");
                                break;
                            case 10:
                                strcpy(targetP->sensorUnits, "ft/min");
                                break;
                            }
                        }
                        else
                        {
                            return COAP_400_BAD_REQUEST;
                        }
                    }
                    else
                    {
                        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                        if (result == 0)
                        {
                            switch (reg_buf[0])
                            {
                            case 0:
                                memset(targetP->sensorUnits, 0x00, 7);
                                break;
                            case 3:
                            case 4:
                            case 5:
                            case 6:
                            case 9:
                                strcpy(targetP->sensorUnits, "kPa");
                                break;
                            case 10:
                                strcpy(targetP->sensorUnits, "m/min");
                                break;
                            }
                        }
                        else
                        {
                            return COAP_400_BAD_REQUEST;
                        }
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
        *dataArrayP = lwm2m_data_new(5);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 5;
        (*dataArrayP)[0].id = 1;
        (*dataArrayP)[1].id = 2;
        (*dataArrayP)[2].id = 3;
        (*dataArrayP)[3].id = 4014;
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
            case 4014:
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
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->logType)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->logType;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 401, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 2:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->logFrequency)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->logFrequency >> 16;
            reg_buf[1] = targetP->logFrequency;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 402, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 3:
        case 4014:
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
    case 4014:
    case 5701:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30012_object(lwm2m_object_t * object)
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

lwm2m_object_t * get_30012_object(void)
{
    lwm2m_object_t * obj30012;

    obj30012 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30012)
    {
        int i;
        prv_instance_t * targetP;

        memset(obj30012, 0, sizeof(lwm2m_object_t));

        obj30012->objID = PLC_DEVICE_LOG;
        for (i = 0; i < 2; i++)
        {
            targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
            if (NULL == targetP) return NULL;
            memset(targetP, 0, sizeof(prv_instance_t));
            targetP->shortID                        = i;
            targetP->logType                        = 0;
            targetP->logFrequency                   = 0;
            memset(targetP->logData, 0x00, MAX_DEVICE_LOG_B);
            memset(targetP->sensorUnits, 0x00, 7);
            obj30012->instanceList = LWM2M_LIST_ADD(obj30012->instanceList, targetP);
        }
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30012->readFunc = prv_read;
        obj30012->discoverFunc = prv_discover;
        obj30012->writeFunc = prv_write;
        obj30012->executeFunc = prv_exec;
        obj30012->createFunc = prv_create;
        obj30012->deleteFunc = prv_delete;
        obj30012->userData = NULL;    // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30012;
}

void free_30012_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

