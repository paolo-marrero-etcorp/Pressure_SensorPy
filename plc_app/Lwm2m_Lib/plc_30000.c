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

/*
 * Implements an object for testing purpose
 *
 *                  Multiple
 * Object |  ID   | Instances | Mandatory |
 *  Test  | 31024 |    Yes    |    No     |
 *
 *  Resources:
 *              Supported    Multiple
 *  Name | ID | Operations | Instances | Mandatory |  Type   | Range | Units | Description |
 *  test |  1 |    R/W     |    No     |    Yes    | Integer | 0-255 |       |             |
 *  exec |  2 |     E      |    No     |    Yes    |         |       |       |             |
 *  dec  |  3 |    R/W     |    No     |    Yes    |  Float  |       |       |             |
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
    struct _prv_instance_ * next;   // matches lwm2m_list_t::next
    uint16_t shortID;               // matches lwm2m_list_t::id
    uint8_t  modbusStnAddr;
    char     softwareVer[16];
    char *   softwareBuildVar;
    char     serialNum[17];
    uint8_t  restartController;  // placeholder for executable
    uint8_t  controllerUnits;
    uint64_t controllerTimeSetting; // seconds since Jan 1, 2000
    uint8_t  optType;
    uint8_t  rstPlungerCycleLog;   // placeholder for executable
    uint8_t  rstDailyProdLog;      // as above
    uint8_t  rstTotalProdLog;      // as above
    uint8_t  rstErrorLog;          // as above
    uint8_t  rstDeviceLog1;         // as above
    uint8_t  rstDeviceLog2;         // as above
    uint32_t dailyLogStartTime;    
    uint8_t  daylightSavingsTimeConfig;
    uint8_t  controllerErrorLog[MAX_ERROR_LOG_B];
    char *   applicationType;      // resource #5750
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
uint8_t poll_30000_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];   

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, 0);
    /* This part is needed to trigger the watcher of the observable */
    if (lwm2m_stringToUri("/30000/0/7", (sizeof("/30000/0/7") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 6, 2, reg_buf);
        if (result == 0)
        {
            targetP->controllerTimeSetting = reg_buf[0];
            targetP->controllerTimeSetting <<= 16;
            targetP->controllerTimeSetting += reg_buf[1];
            lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
            fprintf(stderr, "Controller time: %d\n", targetP->controllerTimeSetting);
#endif    
        }
    }   
    if (lwm2m_stringToUri("/30000/0/1", (sizeof("/30000/0/1") - 1), &uri))
    {
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Modbus Station ID: %d\n", targetP->modbusStnAddr);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/2", (sizeof("/30000/0/2") - 1), &uri))
    {
        result = read_input(FRIENDLY_NAME, modbus_stn_id, 3, 3, reg_buf);
        if (result == 0)
        {
            memset(targetP->softwareVer, 0x00, 16);
            strncpy(targetP->softwareVer, "FW ", 3);
            sprintf(char_buf, "%d", reg_buf[0]);
            strcat(targetP->softwareVer, char_buf);
            strcat(targetP->softwareVer, ".");
            sprintf(char_buf, "%d", reg_buf[1]);
            strcat(targetP->softwareVer, char_buf);
            strcat(targetP->softwareVer, "."); 
            sprintf(char_buf, "%d", reg_buf[2]);
            strcat(targetP->softwareVer, char_buf);
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Software Version: %s\n", targetP->softwareVer);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/3", (sizeof("/30000/0/3") - 1), &uri))
    {
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Software Build Variant: %s\n", targetP->softwareBuildVar);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/4", (sizeof("/30000/0/4") - 1), &uri))
    {
        result = read_input(FRIENDLY_NAME, modbus_stn_id, 1, 1, reg_buf);
        if (result == 0)
        {       
            memset(targetP->serialNum, 0x00, 17);
            sprintf(targetP->serialNum, "%d", reg_buf[0]);
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Serial Number: %s\n", targetP->serialNum);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/6", (sizeof("/30000/0/6") - 1), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            targetP->controllerUnits = char_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Controller Units: %d\n", targetP->controllerUnits);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/8", (sizeof("/30000/0/8") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 201, 1, reg_buf);
        if (result == 0)
        {
            targetP->optType = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Optimization Type: %d\n", targetP->optType);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/15", (sizeof("/30000/0/15") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 13, 2, reg_buf);
        if (result == 0)
        {             
            targetP->dailyLogStartTime = reg_buf[0];
            targetP->dailyLogStartTime <<= 16;
            targetP->dailyLogStartTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Daily Log Start Time: %d\n", targetP->dailyLogStartTime);
#endif    
    }
    if (lwm2m_stringToUri("/30000/0/16", (sizeof("/30000/0/16") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 12, 1, reg_buf);
        if (result == 0)
        {
            targetP->daylightSavingsTimeConfig = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Daylight Savings Time Config: %d\n", targetP->daylightSavingsTimeConfig);
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
    unsigned short log_buf[MAX_ERROR_LOG_W];
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(11);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 11;
        (*dataArrayP)[0].id = 1;
        lwm2m_data_encode_int(targetP->modbusStnAddr, *dataArrayP + 0);
        (*dataArrayP)[1].id = 2;
        lwm2m_data_encode_string(targetP->softwareVer, *dataArrayP + 1);
        (*dataArrayP)[2].id = 3;
        lwm2m_data_encode_string(targetP->softwareBuildVar, *dataArrayP + 2);
        (*dataArrayP)[3].id = 4;
        lwm2m_data_encode_string(targetP->serialNum, *dataArrayP + 3);
        (*dataArrayP)[4].id = 6;
        lwm2m_data_encode_int(targetP->controllerUnits, *dataArrayP + 4);
        (*dataArrayP)[5].id = 7;
        lwm2m_data_encode_int((targetP->controllerTimeSetting + EPOCH_TIME_ADJ), *dataArrayP + 5);
        (*dataArrayP)[6].id = 8;
        lwm2m_data_encode_int(targetP->optType, *dataArrayP + 6);
        (*dataArrayP)[7].id = 15;
        lwm2m_data_encode_int(targetP->dailyLogStartTime, *dataArrayP + 7);
        (*dataArrayP)[8].id = 16;
        lwm2m_data_encode_int(targetP->daylightSavingsTimeConfig, *dataArrayP + 8);
        (*dataArrayP)[9].id = 17;
        lwm2m_data_encode_int(targetP->daylightSavingsTimeConfig, *dataArrayP + 9);
        (*dataArrayP)[10].id = 5750; //applicationType = 5750
        lwm2m_data_encode_string(targetP->applicationType, *dataArrayP + 10);
    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                lwm2m_data_encode_int(targetP->modbusStnAddr, *dataArrayP + i);
                break;
            case 2:
                result = read_input(FRIENDLY_NAME, modbus_stn_id, 3, 3, reg_buf);
                if (result == 0)
                {
                    memset(targetP->softwareVer, 0x00, 16);
                    strncpy(targetP->softwareVer, "FW ", 3);
                    sprintf(char_buf, "%d", reg_buf[0]);
                    strcat(targetP->softwareVer, char_buf);
                    strcat(targetP->softwareVer, ".");
                    sprintf(char_buf, "%d", reg_buf[1]);
                    strcat(targetP->softwareVer, char_buf);
                    strcat(targetP->softwareVer, "."); 
                    sprintf(char_buf, "%d", reg_buf[2]);
                    strcat(targetP->softwareVer, char_buf);   
                    lwm2m_data_encode_string(targetP->softwareVer, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
                break;
            case 3:
                lwm2m_data_encode_string(targetP->softwareBuildVar, *dataArrayP + i);
                break;
            case 4:
                result = read_input(FRIENDLY_NAME, modbus_stn_id, 1, 1, reg_buf);
                if (result == 0)
                {       
                    memset(targetP->serialNum, 0x00, 17);
                    sprintf(targetP->serialNum, "%d", reg_buf[0]);
                    lwm2m_data_encode_string(targetP->serialNum, *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }             
                break;
            case 5:
                return COAP_405_METHOD_NOT_ALLOWED;
            case 6:
                result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
                if (result == 0)
                {
                    targetP->controllerUnits = char_buf[0];
                    lwm2m_data_encode_int(targetP->controllerUnits, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                 
                break;
            case 7:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 6, 2, reg_buf);
                if (result == 0)
                {
                    targetP->controllerTimeSetting = reg_buf[0];
                    targetP->controllerTimeSetting <<= 16;
                    targetP->controllerTimeSetting += reg_buf[1];
                    lwm2m_data_encode_int((targetP->controllerTimeSetting + EPOCH_TIME_ADJ), *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
                break;
            case 8:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 201, 1, reg_buf);
                if (result == 0)
                {
                    targetP->optType = reg_buf[0];
                    lwm2m_data_encode_int(targetP->optType, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
                return COAP_405_METHOD_NOT_ALLOWED;
                break;
            case 15:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 13, 2, reg_buf);
                if (result == 0)
                {             
                    targetP->dailyLogStartTime = reg_buf[0];
                    targetP->dailyLogStartTime <<= 16;
                    targetP->dailyLogStartTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->dailyLogStartTime, *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }             
                break;
            case 16:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 12, 1, reg_buf);
                if (result == 0)
                {
                    targetP->daylightSavingsTimeConfig = reg_buf[0];
                    lwm2m_data_encode_int(targetP->daylightSavingsTimeConfig, *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 17:  
                modbusReg = 2012;
                result = read_input(FRIENDLY_NAME, modbus_stn_id, 2011, 1, reg_buf);
                if ((result == 0) && (reg_buf[0] != 0))
                {
                    numModbusRegs = reg_buf[0];
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
                result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, numModbusRegs, log_buf);
                if (result == 0)
                {
                    j = 0;
                    for (n = 0; n < numModbusRegs; n++)
                    {
                        targetP->controllerErrorLog[j] = log_buf[i] >> 8;
                        j++;
                        targetP->controllerErrorLog[j] = log_buf[i];
                        j++;
                    }
                    lwm2m_data_encode_opaque(targetP->controllerErrorLog, j, *dataArrayP + i);
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
                break;
            case 5750:
                lwm2m_data_encode_string(targetP->applicationType, *dataArrayP + i);
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
        *dataArrayP = lwm2m_data_new(17);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 17;
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
        (*dataArrayP)[15].id = 16;
        (*dataArrayP)[16].id = 17;
        (*dataArrayP)[17].id = 5750;
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
            case 16:
            case 17:
            case 5750:
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
    unsigned char buf[] = { 1, 0 };
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 1:
            {
                int64_t value;

                if (1 != lwm2m_data_decode_int(dataArray + i, &value) || value < 0 || value > 0xFF)
                {
                    return COAP_400_BAD_REQUEST;
                }
                targetP->modbusStnAddr = (uint8_t)value;
                modbus_stn_id = (uint8_t)value;
                set_modbus_address(modbus_stn_id);
            }
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            return COAP_405_METHOD_NOT_ALLOWED;
            break;
        case 6:
            {
                int64_t value;
                
                if (1 != lwm2m_data_decode_int(dataArray + i, &value) || value < 0 || value > 1)
                {
                    return COAP_400_BAD_REQUEST;
                }
                if (value == 1)
                {
                    result = write_coil(FRIENDLY_NAME, modbus_stn_id, 10, buf, 1);    
                }
                else
                {
                    result = write_coil(FRIENDLY_NAME, modbus_stn_id, 10, &buf[1], 1);
                }
                if (result == 0)
                {
                    targetP->controllerUnits = (uint8_t)value;    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;    
                }
            }
            break;
        case 7:
            {
                int64_t value;

                if (1 != lwm2m_data_decode_int(dataArray + i, &value) || value < 0)
                {
                    return COAP_400_BAD_REQUEST;
                }
                value -= EPOCH_TIME_ADJ;
                reg_buf[0] = value >> 16;
                reg_buf[1] = value;
                result = write_holding(FRIENDLY_NAME, modbus_stn_id, 6, reg_buf, 2);
                if (result != 0)
                {
                    return COAP_400_BAD_REQUEST;
                } 
                targetP->controllerTimeSetting = value;
            }
            break;
        case 8:
            {
                int64_t value;

                if (1 != lwm2m_data_decode_int(dataArray + i, &value) || value < 0 || value > 4)
                {
                    return COAP_400_BAD_REQUEST;
                }
                reg_buf[0] = value;
                result = write_holding(FRIENDLY_NAME, modbus_stn_id, 201, reg_buf, 1);
                if (result == 0)
                {
                    targetP->optType = (uint8_t)value;    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;    
                }
            }
        break;
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
            return  COAP_405_METHOD_NOT_ALLOWED;
        case 15:
            {
                int64_t value;
                if (1 != lwm2m_data_decode_int(dataArray + i, &value) || value < 0 || value > 86400)
                {
                    return COAP_400_BAD_REQUEST;
                }
                reg_buf[0] = value >> 16;
                reg_buf[1] = value;
                result = write_holding(FRIENDLY_NAME, modbus_stn_id, 13, reg_buf, 2);
                if (result == 0)
                {
                    targetP->dailyLogStartTime = value;    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;    
                }
            }
            break;
        case 16:
            {
                int64_t value;

                if (1 != lwm2m_data_decode_int(dataArray + i, &value) || value < 0 || value > 1)
                {
                    return COAP_400_BAD_REQUEST;
                }
                reg_buf[0] = value;
                result = write_holding(FRIENDLY_NAME, modbus_stn_id, 12, reg_buf, 1);
                if (result == 0)
                {
                    targetP->daylightSavingsTimeConfig = (uint8_t)value;    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;    
                }  
            }
            break;
        case 17:
            return COAP_405_METHOD_NOT_ALLOWED;
        case 5750:  // res == 5750
            {
                int16_t len;
                len = (dataArray)->value.asBuffer.length;
                if (len == 0)
                {
                    return COAP_400_BAD_REQUEST;
                }
                if (targetP->applicationType != NULL)
                {
                    lwm2m_free(targetP->applicationType);
                }
                targetP->applicationType = (char *)lwm2m_malloc(len + 1);
                memcpy(targetP->applicationType, (dataArray)->value.asBuffer.buffer, len);
                targetP->applicationType[len] = 0x00;
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
    case 2:
    case 3:
    case 4:
        return COAP_405_METHOD_NOT_ALLOWED;
    case 5:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 3, buf, 1);
        return COAP_204_CHANGED;
    case 6:
    case 7:
    case 8:
        return COAP_405_METHOD_NOT_ALLOWED;
    case 9:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 4, buf, 1);
        return COAP_204_CHANGED;
    case 10:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 5, buf, 1);
        return COAP_204_CHANGED;
    case 11:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 11, buf, 1);
        return COAP_204_CHANGED;
    case 12:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 6, buf, 1);
        return COAP_204_CHANGED;
    case 13:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 19, buf, 1);
        return COAP_204_CHANGED;
    case 14:
        result = write_coil(FRIENDLY_NAME, modbus_stn_id, 20, buf, 1);
        return COAP_204_CHANGED;
    case 15:
    case 16:
    case 17:
    case 5750:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30000_object(lwm2m_object_t * object)
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

lwm2m_object_t * get_30000_object(void)
{
    lwm2m_object_t * obj30000;

    obj30000 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30000)
    {
        int i;
        prv_instance_t * targetP;

        memset(obj30000, 0, sizeof(lwm2m_object_t));
        modbus_stn_id = get_modbus_address();
        obj30000->objID = PLUNGER_LIFT_CONTROLLER;
        targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
        if (NULL == targetP) return NULL;
        memset(targetP, 0, sizeof(prv_instance_t));
        targetP->shortID                    = 0;
        targetP->modbusStnAddr              = modbus_stn_id;
        memset(targetP->softwareVer, 0x00, 16);
        targetP->softwareBuildVar           = NULL;
        memset(targetP->serialNum, 0x00, 17);
        targetP->restartController          = 0;
        targetP->controllerUnits            = 0;
        targetP->controllerTimeSetting      = 0;
        targetP->optType                    = 0;
        targetP->rstPlungerCycleLog         = 0;
        targetP->rstDailyProdLog            = 0;
        targetP->rstTotalProdLog            = 0;
        targetP->rstErrorLog                = 0;
        targetP->rstDeviceLog1              = 0;
        targetP->rstDeviceLog2              = 0;
        targetP->dailyLogStartTime          = 0;
        targetP->daylightSavingsTimeConfig  = 0;
        memset(targetP->controllerErrorLog, 0x00, MAX_ERROR_LOG_B);
        targetP->applicationType            = NULL;
        obj30000->instanceList = LWM2M_LIST_ADD(obj30000->instanceList, targetP);
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30000->readFunc = prv_read;
        obj30000->discoverFunc = prv_discover;
        obj30000->writeFunc = prv_write;
        obj30000->executeFunc = prv_exec;
        obj30000->createFunc = prv_create;
        obj30000->deleteFunc = prv_delete;
        obj30000->userData = NULL;   // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30000;
}

uint8_t get_modbus_address(void)
{
    FILE *fp;
    int fileSize;
    //char *fileBuffer;
    char fileBuffer[256];
    char *char_ptr;
    char stnId[4];
    int i;
    char default_config[] = "config={\"ModbusStationAddress\":1}\n";
    
    fp = fopen(CONFIG_FILE, "rb");
    if (!fp) 
    {
        printf("CONFIG_FILE does not exist, creating ...");
        fp = fopen(CONFIG_FILE, "w+");
        fwrite(default_config, strlen(default_config), 1, fp);
        fclose(fp);
        return 1;
    }
    else
    {
        fseek(fp, 0L, SEEK_END);
        fileSize = ftell(fp);
        rewind(fp);
        /* allocate memory for entire content */
        //fileBuffer = calloc(1, fileSize + 1);
        //fileBuffer[fileSize] = 0x00;  // null terminate
        memset(fileBuffer, 0x00, 256);
        //if (!fileBuffer) fclose(fp), fputs("memory alloc fails", stderr), exit(1);
        /* copy the file into the buffer */
        fread(fileBuffer, fileSize, 1, fp);
        /* do your work here, buffer is a string contains the whole text */
        char_ptr = strstr(fileBuffer, "ModbusStationAddress");
        char_ptr += 22;
        i = 0;
        while (*char_ptr != '\n')
        {
            if (isdigit(*char_ptr))
            {
                stnId[i] = *char_ptr;
                i++;
                char_ptr++;
            }
            else
            {
                stnId[i] = 0x00;
                break;
            }
        }
        fclose(fp);
        //free(fileBuffer);
        i = atoi(stnId);
        if (i >= 1)
        {
            return (uint8_t)i;
        }
        else
        {
            return 1;
        }
    }
}

void set_modbus_address(uint8_t id)
{
    FILE *fp;
    char stnId[6];
    char default_config[100] = "config={\"ModbusStationAddress\":";    
    if (id == 0 || id > 255)
        return;
    sprintf(stnId, "%d", id);
    strcat(stnId, "}\n");
    strcat(default_config, stnId);
    fp = fopen(CONFIG_FILE, "w+");
    fwrite(default_config, strlen(default_config), 1, fp);
    fclose(fp);
}

void free_30000_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

