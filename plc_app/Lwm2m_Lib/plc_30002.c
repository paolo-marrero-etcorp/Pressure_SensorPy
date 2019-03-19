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

/* August 2, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller State
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller        | 30002 |    No     |    No     |
 *  Time and Velocity Settings
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             | ID | Operations | Instances | Mandatory |  Type   | Range |  Units         | Description |
 *  Close Velocity                   |  1 |     RW     |    No     |    No     | Float   |       | Units Resource |             |
 *  Close Time                       |  2 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Max Close Time                   |  3 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Min Close Time                   |  4 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Extended Close                   |  5 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Non Arrival Close Time           |  6 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Rise Velocity                    |  7 |     RW     |    No     |    No     | Float   |       | Units Resource |             |
 *  Rise Time                        |  8 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Target Rise Velocity             |  9 |     RW     |    No     |    No     | Float   |       | Units Resource |             |
 *  Target Rise Time                 | 10 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Target Surface Velocity          | 11 |     RW     |    No     |    No     | Float   |       | Units Resource |             |
 *  Afterflow Time                   | 12 |     RW     |    No     |    No     | Integer |       | seconds        |             |                                                                                                 
 *  Max Afterflow                    | 13 |     RW     |    No     |    No     | Integer |       | seconds        |             |                                                                                                        
 *  Min Afterflow                    | 14 |     RW     |    No     |    No     | Integer |       | seconds        |             |     
 *  Extended Afterflow               | 15 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Fast Trip Velocity               | 16 |     RW     |    No     |    No     | Float   |       | Units Resource |             |
 *  Fast Trip Time                   | 17 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Danger Velocity                  | 18 |     RW     |    No     |    No     | Float   |       | Units Resource |             |
 *  Danger Time                      | 19 |     RW     |    No     |    No     | Integer |       | seconds        |             |
 *  Danger/Fast Velocity Source      | 20 |     RW     |    No     |    No     | Integer |       | see manual for enum types    |
 *  Units                            | 5701 |   R      |    No     |    No     | String  |       | m/min or ft/min              |
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
    double      closeVelocity;
    uint64_t    closeTime;
    uint64_t    maxCloseTime;
    uint64_t    minCloseTime;
    uint64_t    extendedClose;
    uint64_t    nonArrivalCloseTime;
    double      riseVelocity;
    uint64_t    riseTime;
    double      targetRiseVelocity;
    uint64_t    targetRiseTime;
    double      targetSurfaceVelocity;
    uint64_t    afterflowTime;
    uint64_t    maxAfterflow;
    uint64_t    minAfterflow;
    uint64_t    extendedAfterflow;
    double      fastTripVelocity;
    uint64_t    fastTripTime;
    double      dangerVelocity;
    uint64_t    dangerTime;
    uint64_t    dangerFastVelSource;
    char        units[7];           // Resource 5701: Read from 0:0010, send "ft/min" = 0 or "m/min" = 1 accordingly
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
uint8_t poll_30002_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];   

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, 0);
    /* This part is needed to trigger the watcher of the observable */
    if (lwm2m_stringToUri("/30002/0/1", (sizeof("/30002/0/1") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 96, 1, reg_buf);
        if (result == 0)
        {
            targetP->closeVelocity = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Close Velocity: %f\n", targetP->closeVelocity);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/2", (sizeof("/30002/0/2") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 143, 2, reg_buf);
        if (result == 0)
        {
            targetP->closeTime = reg_buf[0];
            targetP->closeTime <<= 16;
            targetP->closeTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Close Time: %d\n", targetP->closeTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/3", (sizeof("/30003/0/2") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 137, 2, reg_buf);
        if (result == 0)
        {
            targetP->maxCloseTime = reg_buf[0];
            targetP->maxCloseTime <<= 16;
            targetP->maxCloseTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Max Close Time: %d\n", targetP->maxCloseTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/4", (sizeof("/30003/0/4") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 101, 2, reg_buf);
        if (result == 0)
        {
            targetP->minCloseTime = reg_buf[0];
            targetP->minCloseTime <<= 16;
            targetP->minCloseTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Min Close Time: %d\n", targetP->minCloseTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/5", (sizeof("/30003/0/5") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 134, 2, reg_buf);
        if (result == 0)
        {
            targetP->extendedClose = reg_buf[0];
            targetP->extendedClose <<= 16;
            targetP->extendedClose += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Extended Close: %d\n", targetP->extendedClose);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/6", (sizeof("/30003/0/6") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 104, 2, reg_buf);
        if (result == 0)
        {
            targetP->nonArrivalCloseTime = reg_buf[0];
            targetP->nonArrivalCloseTime <<= 16;
            targetP->nonArrivalCloseTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Non-Arrival Close Time: %d\n", targetP->nonArrivalCloseTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/7", (sizeof("/30003/0/7") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 94, 1, reg_buf);
        if (result == 0)
        {
            targetP->riseVelocity = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Rise Velocity: %f\n", targetP->riseVelocity);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/8", (sizeof("/30003/0/8") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 107, 2, reg_buf);
        if (result == 0)
        {
            targetP->riseTime = reg_buf[0];
            targetP->riseTime <<= 16;
            targetP->riseTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Rise Time: %d\n", targetP->riseTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/9", (sizeof("/30003/0/9") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 95, 1, reg_buf);
        if (result == 0)
        {
            targetP->targetRiseVelocity = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Target Rise Velocity: %f\n", targetP->targetRiseVelocity);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/10", (sizeof("/30003/0/10") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 116, 2, reg_buf);
        if (result == 0)
        {
            targetP->targetRiseTime = reg_buf[0];
            targetP->targetRiseTime <<= 16;
            targetP->targetRiseTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Target Rise Time: %d\n", targetP->targetRiseTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/11", (sizeof("/30003/0/11") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 89, 1, reg_buf);
        if (result == 0)
        {
            targetP->targetSurfaceVelocity = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Target Surface Velocity: %f\n", targetP->targetSurfaceVelocity);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/12", (sizeof("/30003/0/12") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 146, 2, reg_buf);
        if (result == 0)
        {
            targetP->afterflowTime = reg_buf[0];
            targetP->afterflowTime <<= 16;
            targetP->afterflowTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Afterflow time: %d\n", targetP->afterflowTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/13", (sizeof("/30003/0/13") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 131, 2, reg_buf);
        if (result == 0)
        {
            targetP->maxAfterflow = reg_buf[0];
            targetP->maxAfterflow <<= 16;
            targetP->maxAfterflow += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "Max Afterflow: %d\n", targetP->maxAfterflow);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/14", (sizeof("/30003/0/14") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 122, 2, reg_buf);
        if (result == 0)
        {
            targetP->minAfterflow = reg_buf[0];
            targetP->minAfterflow <<= 16;
            targetP->minAfterflow += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "minAfterflow: %d\n", targetP->minAfterflow);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/15", (sizeof("/30003/0/15") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 128, 2, reg_buf);
        if (result == 0)
        {
            targetP->extendedAfterflow = reg_buf[0];
            targetP->extendedAfterflow <<= 16;
            targetP->extendedAfterflow += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "extendedAfterflow: %d\n", targetP->extendedAfterflow);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/16", (sizeof("/30003/0/16") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 93, 1, reg_buf);
        if (result == 0)
        {
            targetP->fastTripVelocity = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "fastTripVelocity: %f\n", targetP->fastTripVelocity);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/17", (sizeof("/30003/0/17") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 113, 2, reg_buf);
        if (result == 0)
        {
            targetP->fastTripTime = reg_buf[0];
            targetP->fastTripTime <<= 16;
            targetP->fastTripTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "fastTripTime: %d\n", targetP->fastTripTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/18", (sizeof("/30003/0/18") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 97, 1, reg_buf);
        if (result == 0)
        {
            targetP->dangerVelocity = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "dangerVelocity: %f\n", targetP->dangerVelocity);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/19", (sizeof("/30003/0/19") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 98, 2, reg_buf);
        if (result == 0)
        {
            targetP->dangerTime = reg_buf[0];
            targetP->dangerTime <<= 16;
            targetP->dangerTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "dangerTime: %d\n", targetP->dangerTime);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/20", (sizeof("/30003/0/20") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 90, 1, reg_buf);
        if (result == 0)
        {
            targetP->dangerFastVelSource = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "dangerFastVelSource: %d\n", targetP->dangerFastVelSource);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/5701", (sizeof("/30003/0/5701") - 1), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->units, 0x00, 7);
            if (char_buf[0] == 0x00)
            {
                strcpy(targetP->units, "ft/min");
            }
            else
            {
                strcpy(targetP->units, "m/min");
            } 
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "units: %s\n", targetP->units);
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
        *dataArrayP = lwm2m_data_new(21);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 21;
        (*dataArrayP)[0].id = 1;
        lwm2m_data_encode_float(targetP->closeVelocity, *dataArrayP + 0);
        (*dataArrayP)[1].id = 2;
        lwm2m_data_encode_int(targetP->closeTime, *dataArrayP + 1);
        (*dataArrayP)[2].id = 3;
        lwm2m_data_encode_int(targetP->maxCloseTime, *dataArrayP + 2);
        (*dataArrayP)[3].id = 4;
        lwm2m_data_encode_int(targetP->minCloseTime, *dataArrayP + 3);
        (*dataArrayP)[4].id = 5;
        lwm2m_data_encode_int(targetP->extendedClose, *dataArrayP + 4);
        (*dataArrayP)[5].id = 6;
        lwm2m_data_encode_int(targetP->nonArrivalCloseTime, *dataArrayP + 5);
        (*dataArrayP)[6].id = 7;
        lwm2m_data_encode_float(targetP->riseVelocity, *dataArrayP + 6);
        (*dataArrayP)[7].id = 8;
        lwm2m_data_encode_int(targetP->riseTime, *dataArrayP + 7);
        (*dataArrayP)[8].id = 9;
        lwm2m_data_encode_float(targetP->targetRiseVelocity, *dataArrayP + 8);
        (*dataArrayP)[9].id = 10; 
        lwm2m_data_encode_int(targetP->targetRiseTime, *dataArrayP + 9);
        (*dataArrayP)[10].id = 11; 
        lwm2m_data_encode_float(targetP->targetSurfaceVelocity, *dataArrayP + 10);   
        (*dataArrayP)[11].id = 12; 
        lwm2m_data_encode_int(targetP->afterflowTime, *dataArrayP + 11);
        (*dataArrayP)[12].id = 13; 
        lwm2m_data_encode_int(targetP->maxAfterflow, *dataArrayP + 12);
        (*dataArrayP)[13].id = 14; 
        lwm2m_data_encode_int(targetP->minAfterflow, *dataArrayP + 13);
        (*dataArrayP)[14].id = 15; 
        lwm2m_data_encode_int(targetP->extendedAfterflow, *dataArrayP + 14);  
        (*dataArrayP)[15].id = 16; 
        lwm2m_data_encode_float(targetP->fastTripVelocity, *dataArrayP + 15);
        (*dataArrayP)[16].id = 17; 
        lwm2m_data_encode_int(targetP->fastTripTime, *dataArrayP + 16);
        (*dataArrayP)[17].id = 18; 
        lwm2m_data_encode_float(targetP->dangerVelocity, *dataArrayP + 17);
        (*dataArrayP)[18].id = 19; 
        lwm2m_data_encode_int(targetP->dangerTime, *dataArrayP + 18);        
        (*dataArrayP)[19].id = 20; 
        lwm2m_data_encode_int(targetP->dangerFastVelSource, *dataArrayP + 19);
        (*dataArrayP)[20].id = 5701; // resource 5701
        lwm2m_data_encode_string(targetP->units, *dataArrayP + 20);
    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 96, 1, reg_buf);
                if (result == 0)
                {
                    targetP->closeVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->closeVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 2:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 143, 2, reg_buf);
                if (result == 0)
                {
                    targetP->closeTime = reg_buf[0];
                    targetP->closeTime <<= 16;
                    targetP->closeTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->closeTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 3:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 137, 2, reg_buf);
                if (result == 0)
                {
                    targetP->maxCloseTime = reg_buf[0];
                    targetP->maxCloseTime <<= 16;
                    targetP->maxCloseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->maxCloseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 4:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 101, 2, reg_buf);
                if (result == 0)
                {
                    targetP->minCloseTime = reg_buf[0];
                    targetP->minCloseTime <<= 16;
                    targetP->minCloseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->minCloseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 134, 2, reg_buf);
                if (result == 0)
                {
                    targetP->extendedClose = reg_buf[0];
                    targetP->extendedClose <<= 16;
                    targetP->extendedClose += reg_buf[1];
                    lwm2m_data_encode_int(targetP->extendedClose, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 6:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 104, 2, reg_buf);
                if (result == 0)
                {
                    targetP->nonArrivalCloseTime = reg_buf[0];
                    targetP->nonArrivalCloseTime <<= 16;
                    targetP->nonArrivalCloseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->nonArrivalCloseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 7:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 94, 1, reg_buf);
                if (result == 0)
                {
                    targetP->riseVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->riseVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 8:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 107, 2, reg_buf);
                if (result == 0)
                {
                    targetP->riseTime = reg_buf[0];
                    targetP->riseTime <<= 16;
                    targetP->riseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->riseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 9:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 95, 1, reg_buf);
                if (result == 0)
                {
                    targetP->targetRiseVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->targetRiseVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;             
            case 10:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 116, 2, reg_buf);
                if (result == 0)
                {
                    targetP->targetRiseTime = reg_buf[0];
                    targetP->targetRiseTime <<= 16;
                    targetP->targetRiseTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->targetRiseTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 11:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 89, 1, reg_buf);
                if (result == 0)
                {
                    targetP->targetSurfaceVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->targetSurfaceVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;               
            case 12:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 146, 2, reg_buf);
                if (result == 0)
                {
                    targetP->afterflowTime = reg_buf[0];
                    targetP->afterflowTime <<= 16;
                    targetP->afterflowTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->afterflowTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;                
            case 13:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 131, 2, reg_buf);
                if (result == 0)
                {
                    targetP->maxAfterflow = reg_buf[0];
                    targetP->maxAfterflow <<= 16;
                    targetP->maxAfterflow += reg_buf[1];
                    lwm2m_data_encode_int(targetP->maxAfterflow, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;                
            case 14:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 122, 2, reg_buf);
                if (result == 0)
                {
                    targetP->minAfterflow = reg_buf[0];
                    targetP->minAfterflow <<= 16;
                    targetP->minAfterflow += reg_buf[1];
                    lwm2m_data_encode_int(targetP->minAfterflow, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 15:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 128, 2, reg_buf);
                if (result == 0)
                {
                    targetP->extendedAfterflow = reg_buf[0];
                    targetP->extendedAfterflow <<= 16;
                    targetP->extendedAfterflow += reg_buf[1];
                    lwm2m_data_encode_int(targetP->extendedAfterflow, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 16:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 93, 1, reg_buf);
                if (result == 0)
                {
                    targetP->fastTripVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->fastTripVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break; 
            case 17:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 113, 2, reg_buf);
                if (result == 0)
                {
                    targetP->fastTripTime = reg_buf[0];
                    targetP->fastTripTime <<= 16;
                    targetP->fastTripTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->fastTripTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 18:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 97, 1, reg_buf);
                if (result == 0)
                {
                    targetP->dangerVelocity = reg_buf[0];
                    lwm2m_data_encode_float(targetP->dangerVelocity, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;   
            case 19:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 98, 2, reg_buf);
                if (result == 0)
                {
                    targetP->dangerTime = reg_buf[0];
                    targetP->dangerTime <<= 16;
                    targetP->dangerTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->dangerTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;    
            case 20:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 90, 1, reg_buf);
                if (result == 0)
                {
                    targetP->dangerFastVelSource = reg_buf[0];
                    lwm2m_data_encode_float(targetP->dangerFastVelSource, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break; 
            case 21:
            case 5701:
                result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
                if (result == 0)
                {
                    memset(targetP->units, 0x00, 7);
                    if (char_buf[0] == 0x00)
                    {
                        strcpy(targetP->units, "ft/min");
                    }
                    else
                    {
                        strcpy(targetP->units, "m/min");
                    }                        
                    lwm2m_data_encode_string(targetP->units, *dataArrayP + i);
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
        *dataArrayP = lwm2m_data_new(21);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 21;
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
        (*dataArrayP)[17].id = 18;
        (*dataArrayP)[18].id = 19;
        (*dataArrayP)[19].id = 20;
        (*dataArrayP)[20].id = 5701;
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
            case 18:
            case 19:
            case 20:
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
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->closeVelocity)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->closeVelocity;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 96, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 2:
            {
                uint64_t value;
                if (1 != lwm2m_data_decode_int(dataArray + i, &value))
                {
                    return COAP_400_BAD_REQUEST;
                } 
                reg_buf[0] = value >> 16;
                reg_buf[1] = value;
                result = write_holding(FRIENDLY_NAME, modbus_stn_id, 143, reg_buf, 2);
                if (result == 0)
                {
                    targetP->closeTime = value;    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;    
                }
            }
            break;
        case 3:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->maxCloseTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->maxCloseTime >> 16;
            reg_buf[1] = targetP->maxCloseTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 137, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;            
        case 4:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->minCloseTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->minCloseTime >> 16;
            reg_buf[1] = targetP->minCloseTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 101, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;             
        case 5:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->extendedClose)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->extendedClose >> 16;
            reg_buf[1] = targetP->extendedClose;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 134, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 6:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->nonArrivalCloseTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->nonArrivalCloseTime >> 16;
            reg_buf[1] = targetP->nonArrivalCloseTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 104, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 7:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->riseVelocity)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->riseVelocity;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 94, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 8:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->riseTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->riseTime >> 16;
            reg_buf[1] = targetP->riseTime; 
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 107, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 9:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->targetRiseVelocity)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->targetRiseVelocity;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 95, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;                
        case 10:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->targetRiseTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->targetRiseTime >> 16;
            reg_buf[1] = targetP->targetRiseTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 116, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 11:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->targetSurfaceVelocity)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->targetSurfaceVelocity;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 89, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;                 
        case 12:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->afterflowTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->afterflowTime >> 16;
            reg_buf[1] = targetP->afterflowTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 146, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                 
        case 13:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->maxAfterflow)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->maxAfterflow >> 16;
            reg_buf[1] = targetP->maxAfterflow;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 131, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 14:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->minAfterflow)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->minAfterflow >> 16;
            reg_buf[1] = targetP->minAfterflow;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 122, reg_buf, 3);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 15:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->extendedAfterflow)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->extendedAfterflow >> 16;
            reg_buf[1] = targetP->extendedAfterflow;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 128, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 16:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->fastTripVelocity)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->fastTripVelocity;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 93, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break; 
        case 17:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->fastTripTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->fastTripTime >> 16;
            reg_buf[1] = targetP->fastTripTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 113, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 18:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->dangerVelocity)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->dangerVelocity;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 97, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;                
        case 19:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->dangerTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->dangerTime >> 16;
            reg_buf[1] = targetP->dangerTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 98, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 20:  
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->dangerFastVelSource)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->dangerFastVelSource;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 90, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 21:
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
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 5750:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30002_object(lwm2m_object_t * object)
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

lwm2m_object_t * get_30002_object(void)
{
    lwm2m_object_t * obj30002;

    obj30002 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30002)
    {
        int i;
        prv_instance_t * targetP;

        memset(obj30002, 0, sizeof(lwm2m_object_t));

        obj30002->objID = PLC_TIME_VELOCITY_SETTINGS;
        targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
        if (NULL == targetP) return NULL;
        memset(targetP, 0, sizeof(prv_instance_t));
        targetP->shortID                    = 0;
        targetP->closeVelocity              = 0;
        targetP->closeTime                  = 0;
        targetP->maxCloseTime               = 0;
        targetP->minCloseTime               = 0;
        targetP->extendedClose              = 0;
        targetP->nonArrivalCloseTime        = 0;
        targetP->riseVelocity               = 0;
        targetP->riseTime                   = 0;
        targetP->targetRiseVelocity         = 0;
        targetP->targetRiseTime             = 0;
        targetP->targetSurfaceVelocity      = 0;
        targetP->afterflowTime              = 0;
        targetP->maxAfterflow               = 0;
        targetP->minAfterflow               = 0;
        targetP->extendedAfterflow          = 0;
        targetP->fastTripVelocity           = 0;
        targetP->fastTripTime               = 0;
        targetP->dangerVelocity             = 0;
        targetP->dangerTime                 = 0;
        targetP->dangerFastVelSource        = 0;
        memset(targetP->units, 0x00, 7);
        obj30002->instanceList = LWM2M_LIST_ADD(obj30002->instanceList, targetP);
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30002->readFunc = prv_read;
        obj30002->discoverFunc = prv_discover;
        obj30002->writeFunc = prv_write;
        obj30002->executeFunc = prv_exec;
        obj30002->createFunc = prv_create;
        obj30002->deleteFunc = prv_delete;
        obj30002->userData = NULL;   // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30002;
}

void free_30002_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

