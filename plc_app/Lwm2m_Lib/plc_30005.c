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

/* August 27, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller Pressure or Flow Input
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller        | 30005 |  Multiple |    No     |
 *  Pressure or Flow Input
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             | ID | Operations | Instances | Mandatory |  Type   | Range  |  Units         | Description                                    |
 *  Device Configuration             |  1 |     RW     |    No     |    Yes    | Integer | 0-3    |                | 0 = disabled, 1 = Pressure Switch              |
 *                                   |    |            |           |           |         |        |                | 2 = Pressure Sensor, 3 = Pressure virtual*     |
 *  Switch Value                     |  2 |     R      |    No     |    No     | Integer | 0-1    |                | 0 – Switch Reset 1 – Switch Tripped            |
 *  Switch Mode                      |  3 |     RW     |    No     |    No     | Integer | 0-1    |                | 0 = Normally Open 1 = Normally Closed          |
 *  Sensor Range                     |  4 |     RW     |    No     |    No     | Float   |        |                |                                                |     
 *  Sensor Status                    |  5 |     R      |    No     |    No     | Integer | 0-6    |                | 0 - disabled 1 – scan pending 2 - def change   | 
 *                                   |    |            |           |           |         |        |                | pending, 3 – value under range 4 – value over  |
 *                                   |    |            |           |           |         |        |                | range, 5 – value invalid 6 – value valid       |
 *  Virtual Sensor Value             |  6 |     RW     |    No     |    No     | Float   |        |                | This is the virtual sensor value that is       |
 *                                   |    |            |           |           |         |        |                | written in from another device to be used in   | 
 *                                   |    |            |           |           |         |        |                | the controller state engine                    |
 *  Sensor Value                     | 5700 |   R      |    No     |    No     | Float   |        |                |                                                |                                 
 *  Sensor Units                     | 5701 |   R      |    No     |    No     | String  |        |                | Depends on settings on PLC                     |
 *  Application Type                 | 5750 |   RW     |    No     |    Yes    | String  |        |                | E.g. Line Pressure, Casing Pressure, Tubing    | 
 *                                   |      |          |           |           |         |        |                | Pressure, Differential Pressure, Flow          |
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
    uint64_t    deviceConfig;
    uint64_t    switchValue;
    uint64_t    switchMode;
    double      sensorRange;
    uint64_t    sensorStatus;
    double      virtualSensorValue;
    double      sensorValue;
    char        units[7];            // Resource 5701: Read from 0:0010, send "PSI" = 0 or "kPa" = 1 accordingly
    char *      applicationType;       // resource #5750           
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
uint8_t poll_30005_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId)
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
    sprintf(char_uri, "/30005/%d/1", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 241;
            break;
        case 1:
            modbusReg = 331;
            break;
        case 2:
            modbusReg = 261;
            break;
        case 3:
            modbusReg = 291;
            break;
        case 4:
            modbusReg = 311;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }                
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 331, 1, reg_buf);
        if (result == 0)
        {
            targetP->deviceConfig = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "deviceConfig: %d\n", targetP->deviceConfig);
#endif    
    }
    sprintf(char_uri, "/30005/%d/2", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 32;
            break;
        case 1:
            modbusReg = 36;
            break;
        case 2:
            modbusReg = 33;
            break;
        case 3:
            modbusReg = 34;
            break;
        case 4:
            modbusReg = 35;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }                
        result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 36, 1, char_buf);
        if (result == 0)
        {
            targetP->switchValue = char_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "switchValue: %d\n", targetP->switchValue);
#endif    
    }
    sprintf(char_uri, "/30005/%d/3", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 242;
            break;
        case 1:
            modbusReg = 332;
            break;
        case 2:
            modbusReg = 262;
            break;
        case 3:
            modbusReg = 292;
            break;
        case 4:
            modbusReg = 312;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }                
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->switchMode = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }                
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "switchValue: %d\n", targetP->switchValue);
#endif    
    }
    sprintf(char_uri, "/30005/%d/4", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 243;
            break;
        case 1:
            modbusReg = 333;
            break;
        case 2:
            modbusReg = 263;
            break;
        case 3:
            modbusReg = 293;
            break;
        case 4:
            modbusReg = 313;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }                
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->sensorRange = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        } 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "sensorRange: %f\n", targetP->sensorRange);
#endif    
    }
    sprintf(char_uri, "/30005/%d/5", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 52;
            break;
        case 1:
            modbusReg = 56;
            break;
        case 2:
            modbusReg = 53;
            break;
        case 3:
            modbusReg = 54;
            break;
        case 4:
            modbusReg = 55;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }                
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->sensorStatus = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        } 
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "sensorStatus: %d\n", targetP->sensorStatus);
#endif    
    }
    sprintf(char_uri, "/30005/%d/6", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 374;
            break;
        case 1:
            modbusReg = 375;
            break;
        case 2:
            modbusReg = 371;
            break;
        case 3:
            modbusReg = 372;
            break;
        case 4:
            modbusReg = 373;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }                
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->virtualSensorValue = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "virtualSensorValue: %f\n", targetP->virtualSensorValue);
#endif    
    }
    sprintf(char_uri, "/30005/%d/5700", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        switch (instanceId)
        {
        case 0:
            modbusReg = 32;
            break;
        case 1:
            modbusReg = 36;
            break;
        case 2:
            modbusReg = 33;
            break;
        case 3:
            modbusReg = 34;
            break;
        case 4:
            modbusReg = 35;
            break;                    
        default:  
            return COAP_400_BAD_REQUEST;
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->sensorValue = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "sensorValue: %f\n", targetP->sensorValue);
#endif    
    }
    sprintf(char_uri, "/30005/%d/5701", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->units, 0x00, 7);
            if (instanceId < 3)
            {
                if (char_buf[0] == 0x00)
                {
                    strcpy(targetP->units, "PSI");
                }
                else
                {
                    strcpy(targetP->units, "kPa");
                }                         
            }
            else if (instanceId == 3)
            {
                if (char_buf[0] == 0x00)
                {
                    strcpy(targetP->units, "H2O");
                }
                else
                {
                    strcpy(targetP->units, "mbar");
                }                         
            }
            else
            {
                if (char_buf[0] == 0x00)
                {
                    strcpy(targetP->units, "Mcf/d");
                }
                else
                {
                    strcpy(targetP->units, "e3m3/d");
                }                         
            }                       
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }  
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "units: %s\n", targetP->sensorValue);
#endif    
    }
    sprintf(char_uri, "/30005/%d/5750", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "applicationType: %s\n", targetP->applicationType);
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
        *dataArrayP = lwm2m_data_new(9);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 9;
        (*dataArrayP)[0].id = 1;
        lwm2m_data_encode_int(targetP->deviceConfig, *dataArrayP + 0);
        (*dataArrayP)[1].id = 2;
        lwm2m_data_encode_int(targetP->switchValue, *dataArrayP + 1);
        (*dataArrayP)[2].id = 3;
        lwm2m_data_encode_int(targetP->switchMode, *dataArrayP + 2);
        (*dataArrayP)[3].id = 4;
        lwm2m_data_encode_float(targetP->sensorRange, *dataArrayP + 3);
        (*dataArrayP)[4].id = 5;
        lwm2m_data_encode_int(targetP->sensorStatus, *dataArrayP + 4);
        (*dataArrayP)[5].id = 6;
        lwm2m_data_encode_float(targetP->virtualSensorValue, *dataArrayP + 5);
        (*dataArrayP)[6].id = 5700;
        lwm2m_data_encode_float(targetP->sensorValue, *dataArrayP + 6);
        (*dataArrayP)[7].id = 5701; // resource 5701
        lwm2m_data_encode_string(targetP->units, *dataArrayP + 7);
        (*dataArrayP)[8].id = 5750;  // resource 5750
        lwm2m_data_encode_string(targetP->applicationType, *dataArrayP + 8);
    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 241;
                    break;
                case 1:
                    modbusReg = 331;
                    break;
                case 2:
                    modbusReg = 261;
                    break;
                case 3:
                    modbusReg = 291;
                    break;
                case 4:
                    modbusReg = 311;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }                
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 331, 1, reg_buf);
                if (result == 0)
                {
                    targetP->deviceConfig = reg_buf[0];
                    lwm2m_data_encode_int(targetP->deviceConfig, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 2:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 32;
                    break;
                case 1:
                    modbusReg = 36;
                    break;
                case 2:
                    modbusReg = 33;
                    break;
                case 3:
                    modbusReg = 34;
                    break;
                case 4:
                    modbusReg = 35;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }                
                result = read_discretes(FRIENDLY_NAME, modbus_stn_id, 36, 1, char_buf);
                if (result == 0)
                {
                    targetP->switchValue = char_buf[0];
                    lwm2m_data_encode_int(targetP->switchValue, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 3:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 242;
                    break;
                case 1:
                    modbusReg = 332;
                    break;
                case 2:
                    modbusReg = 262;
                    break;
                case 3:
                    modbusReg = 292;
                    break;
                case 4:
                    modbusReg = 312;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }                
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->switchMode = reg_buf[0];
                    lwm2m_data_encode_int(targetP->switchMode, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 4:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 243;
                    break;
                case 1:
                    modbusReg = 333;
                    break;
                case 2:
                    modbusReg = 263;
                    break;
                case 3:
                    modbusReg = 293;
                    break;
                case 4:
                    modbusReg = 313;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }                
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->sensorRange = reg_buf[0];
                    lwm2m_data_encode_float(targetP->sensorRange, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 52;
                    break;
                case 1:
                    modbusReg = 56;
                    break;
                case 2:
                    modbusReg = 53;
                    break;
                case 3:
                    modbusReg = 54;
                    break;
                case 4:
                    modbusReg = 55;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }                
                result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->sensorStatus = reg_buf[0];
                    lwm2m_data_encode_int(targetP->sensorStatus, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 6:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 374;
                    break;
                case 1:
                    modbusReg = 375;
                    break;
                case 2:
                    modbusReg = 371;
                    break;
                case 3:
                    modbusReg = 372;
                    break;
                case 4:
                    modbusReg = 373;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }                
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->virtualSensorValue = reg_buf[0];
                    lwm2m_data_encode_float(targetP->virtualSensorValue, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5700:
                switch (instanceId)
                {
                case 0:
                    modbusReg = 32;
                    break;
                case 1:
                    modbusReg = 36;
                    break;
                case 2:
                    modbusReg = 33;
                    break;
                case 3:
                    modbusReg = 34;
                    break;
                case 4:
                    modbusReg = 35;
                    break;                    
                default:  
                    return COAP_400_BAD_REQUEST;
                }
                result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
                if (result == 0)
                {
                    targetP->sensorValue = reg_buf[0];
                    lwm2m_data_encode_float(targetP->sensorValue, *dataArrayP + i);    
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
                    memset(targetP->units, 0x00, 7);
                    if (instanceId < 3)
                    {
                        if (char_buf[0] == 0x00)
                        {
                            strcpy(targetP->units, "PSI");
                        }
                        else
                        {
                            strcpy(targetP->units, "kPa");
                        }                         
                    }
                    else if (instanceId == 3)
                    {
                        if (char_buf[0] == 0x00)
                        {
                            strcpy(targetP->units, "H2O");
                        }
                        else
                        {
                            strcpy(targetP->units, "mbar");
                        }                         
                    }
                    else
                    {
                        if (char_buf[0] == 0x00)
                        {
                            strcpy(targetP->units, "Mcf/d");
                        }
                        else
                        {
                            strcpy(targetP->units, "e3m3/d");
                        }                         
                    }                       
                    lwm2m_data_encode_string(targetP->units, *dataArrayP + i);    
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
        *dataArrayP = lwm2m_data_new(9);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 21;
        (*dataArrayP)[0].id = 1;
        (*dataArrayP)[1].id = 2;
        (*dataArrayP)[2].id = 3;
        (*dataArrayP)[3].id = 4;
        (*dataArrayP)[4].id = 5;
        (*dataArrayP)[5].id = 6;
        (*dataArrayP)[6].id = 5700;
        (*dataArrayP)[7].id = 5701;
        (*dataArrayP)[8].id = 5750;
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
            case 5700:
            case 5701:
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
    int modbusReg;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 1:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->deviceConfig)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->deviceConfig;
            switch (instanceId)
            {
            case 0:
                modbusReg = 241;
                break;
            case 1:
                modbusReg = 331;
                break;
            case 2:
                modbusReg = 261;
                break;
            case 3:
                modbusReg = 291;
                break;
            case 4:
                modbusReg = 311;
                break;                
            default:
                return COAP_400_BAD_REQUEST;
            }
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 2:
            return COAP_400_BAD_REQUEST;
            break;
        case 3:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->switchMode)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->switchMode;
            switch (instanceId)
            {
            case 0:
                modbusReg = 242;
                break;
            case 1:
                modbusReg = 332;
                break;
            case 2:
                modbusReg = 262;
                break;
            case 3:
                modbusReg = 292;
                break;
            case 4:
                modbusReg = 312;
                break;                
            default:
                return COAP_400_BAD_REQUEST;
            }            
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;            
        case 4:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->sensorRange)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->sensorRange;
            switch (instanceId)
            {
            case 0:
                modbusReg = 243;
                break;
            case 1:
                modbusReg = 333;
                break;
            case 2:
                modbusReg = 263;
                break;
            case 3:
                modbusReg = 293;
                break;
            case 4:
                modbusReg = 313;
                break;                
            default:
                return COAP_400_BAD_REQUEST;
            }            
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;             
        case 5:
            return COAP_400_BAD_REQUEST;
            break;
        case 6:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->virtualSensorValue)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->virtualSensorValue;
            switch (instanceId)
            {
            case 0:
                modbusReg = 374;
                break;
            case 1:
                modbusReg = 375;
                break;
            case 2:
                modbusReg = 371;
                break;
            case 3:
                modbusReg = 372;
                break;
            case 4:
                modbusReg = 373;
                break;                
            default:
                return COAP_400_BAD_REQUEST;
            }            
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, modbusReg, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 7:
        case 5700:  // res == 5700
            return COAP_405_METHOD_NOT_ALLOWED;
        case 8:
        case 5701:  // res == 5701
            return COAP_405_METHOD_NOT_ALLOWED;
        case 9:
        case 5750 :   // res == 5750
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
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 5700:
    case 5701:
    case 5750:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30005_object(lwm2m_object_t * object)
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

lwm2m_object_t * get_30005_object(void)
{
    lwm2m_object_t * obj30005;

    obj30005 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30005)
    {
        int i;
        prv_instance_t * targetP;

        memset(obj30005, 0, sizeof(lwm2m_object_t));

        obj30005->objID = PLC_PRESSURE_OR_FLOW_INPUT;
        for (i = 0; i < 5; i++)
        {
            targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
            if (NULL == targetP) return NULL;
            memset(targetP, 0, sizeof(prv_instance_t));
            targetP->shortID                        = i;
            targetP->deviceConfig                   = 0;
            targetP->switchValue                    = 0;               
            targetP->switchMode                     = 0;
            targetP->sensorRange                    = 0;
            targetP->sensorStatus                   = 0;
            targetP->virtualSensorValue             = 0;
            targetP->sensorValue                    = 0;
            memset(targetP->units, 0x00, 7);
            targetP->applicationType                = NULL;
            obj30005->instanceList = LWM2M_LIST_ADD(obj30005->instanceList, targetP);            
        }
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30005->readFunc = prv_read;
        obj30005->discoverFunc = prv_discover;
        obj30005->writeFunc = prv_write;
        obj30005->executeFunc = prv_exec;
        obj30005->createFunc = prv_create;
        obj30005->deleteFunc = prv_delete;
        obj30005->userData = NULL;   // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30005;
}

void free_30005_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

