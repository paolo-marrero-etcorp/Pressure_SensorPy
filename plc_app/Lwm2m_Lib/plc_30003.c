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

/* August 5, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller Velocity and Pressure Optimization Settings
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller        | 30003 |    No     |    No     |
 *  Velocity and Pressure
 *  Optimization Settings
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             | ID | Operations | Instances | Mandatory |  Type   | Range  |  Units         | Description                     |
 *  Optimization Type                |  1 |     RW     |    No     |    Yes    | Integer |        |                | 0 = disabled, 1 = Pressure Opt  |
 *                                   |    |            |           |           |         |        |                | 2 = Afterflow Timer Opt         |
 *                                   |    |            |           |           |         |        |                | 3 = Close Timer Opt             |
 *                                   |    |            |           |           |         |        |                | 4 = Close the Afterflow Opt     |
 *  Arrival Guard Time               |  2 |     RW     |    No     |    No     | Integer | 1-600  | seconds        |                                 |
 *  Afterflow Scale Factor           |  3 |     RW     |    No     |    No     | Integer | 0-100  |                | 0 - disables adjustments        |
 *                                   |    |            |           |           |         |        |                | Percentage of adjustment        |
 *  Close Scale Factor               |  4 |     RW     |    No     |    No     | Integer | 1-101  |                | 1 - disables adjustments        |     
 *                                   |    |            |           |           |         |        |                | Percentage of adjustment        |
 *  Adjustment Type                  |  5 |     RW     |    No     |    No     | Integer | 0-1    |                | 0 - Max Min, 1 - current time   |
 *  Velocity Source                  |  6 |     RW     |    No     |    No     | Integer | 0-1    |                | 0 = Average Velocity (default)  |
 *                                   |    |            |           |           |         |        |                | 1 = Surface Velocity            |
 *  Open Tubing Trip                 |  7 |     RW     |    No     |    No     | Float   |        |                | Units Resource                  |
 *  Open Tubing Reset                |  8 |     RW     |    No     |    No     | Float   |        |                | Units Resource                  |
 *  Tubing Stable Time               |  9 |     RW     |    No     |    No     | Integer | 0-7199 | seconds        |                                 |
 *  Open Tubing Enable               | 10 |     RW     |    No     |    No     | Boolean |        | seconds        | F = Disabled, T = Enabled       |
 *  Open Casing Trip                 | 11 |     RW     |    No     |    No     | Float   |        |                | see Units Resource              |
 *  Open Casing Reset                | 12 |     RW     |    No     |    No     | Float   |        |                | see Units Resource              |                                                                                                 
 *  Open Casing Stable Time          | 13 |     RW     |    No     |    No     | Integer | 0-7199 | seconds        |                                 |                                                                                                        
 *  Open Casing Enable               | 14 |     RW     |    No     |    No     | Boolean |        |                | F = Disabled, T = Enabled       |     
 *  Open Casing-Line Differential    | 15 |     RW     |    No     |    No     | Float   |        |                | see Units Resource              |
 *  Trip                             |    |            |           |           |         |        |                |                                 |
 *  Open Casing-Line Differential    | 16 |     RW     |    No     |    No     | Float   |        |                | see Units Resource              |
 *  Reset                            |    |            |           |           |         |        |                |                                 |
 *  Open Casing-Line Differential    | 17 |     RW     |    No     |    No     | Integer | 0-7199 | seconds        |                                 |
 *  Stable Time                      |    |            |           |           |         |        |                |                                 |
 *  Open Casing-Line Differential    | 18 |     RW     |    No     |    No     | Boolean |        |                | F = Disabled, T = Enabled       |
 *  Enable                           |    |            |           |           |         |        |                |                                 |
 *  Close Casing Trip                | 19 |     RW     |    No     |    No     | Float   |        |                | see Units Resource              |
 *  Close Casing Reset               | 20 |     RW     |    No     |    No     | Float   |        |                | see Units Resource              |
 *  Close Casing Stable Time         | 21 |     RW     |    No     |    No     | Integer |        | seconds        |                                 |
 *  Close Casing Pressure            |    |            |           |           |         |        |                |                                 |
 *  Threshold per minute             | 22 |     RW     |    No     |    No     | Float   |        |                | see units resource              |
 *  Casing Pressure Rate Trip Delay  | 23 |     RW     |    No     |    No     | Integer | 1-1800000 seconds       |                                 |
 *  Close CP Enable                  | 24 |     RW     |    No     |    No     | Integer | 0-2    | 0 = Disabled, 1 = Rate Drop, 2 = Absolute        |
 *  Units                            | 5701 |   R      |    No     |    No     | String  |        | kPa or PSI                                       |
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
    uint64_t    optimizationType;
    uint64_t    arrivalGuardTime;
    uint64_t    afterflowScaleFactor;
    uint64_t    closeScaleFactor;
    uint64_t    adjustmentType;
    uint64_t    velocitySource;
    double      openTubingTrip;
    double      openTubingReset;
    uint64_t    tubingStableTime;
    bool        openTubingEnable;
    double      openCasingTrip;
    double      openCasingReset;
    uint64_t    openCasingStableTime;
    bool        openCasingEnable;
    double      openCasingLineDiffTrip;
    double      openCasingLineDiffReset;
    uint64_t    openCasingLineDiffStableTime;
    bool        openCasingLineDiffEnable;
    double      closeCasingTrip;
    double      closeCasingReset;
    uint64_t    closeCasingStableTime;
    double      closeCasingPressureThrPerMin;
    uint64_t    casingPressureRateTripDelay;
    uint64_t    closeCPEnable;
    char        units[4];           // Resource 5701: Read from 0:0010, send "PSI" = 0 or "kPa" = 1 accordingly
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
uint8_t poll_30003_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];   

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, 0);
    /* This part is needed to trigger the watcher of the observable */
    if (lwm2m_stringToUri("/30003/0/1", (sizeof("/30003/0/1") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 201, 1, reg_buf);
        if (result == 0)
        {
            targetP->optimizationType = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "optimizationType: %d\n", targetP->optimizationType);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/2", (sizeof("/30003/0/2") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 125, 2, reg_buf);
        if (result == 0)
        {
            targetP->arrivalGuardTime = reg_buf[0];
            targetP->arrivalGuardTime <<= 16;
            targetP->arrivalGuardTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "arrivalGuardTime: %d\n", targetP->arrivalGuardTime);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/3", (sizeof("/30003/0/3") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 202, 1, reg_buf);
        if (result == 0)
        {
            targetP->afterflowScaleFactor = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "afterflowScaleFactor: %d\n", targetP->afterflowScaleFactor);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/4", (sizeof("/30003/0/4") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 204, 1, reg_buf);
        if (result == 0)
        {
            targetP->closeScaleFactor = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "closeScaleFactor: %d\n", targetP->closeScaleFactor);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/5", (sizeof("/30003/0/5") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 206, 1, reg_buf);
        if (result == 0)
        {
            targetP->adjustmentType = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "adjustmentType: %d\n", targetP->adjustmentType);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/6", (sizeof("/30003/0/6") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 200, 1, reg_buf);
        if (result == 0)
        {
            targetP->velocitySource = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "velocitySource: %d\n", targetP->velocitySource);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/7", (sizeof("/30003/0/7") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 334, 1, reg_buf);
        if (result == 0)
        {
            targetP->openTubingTrip = reg_buf[0];
            targetP->openTubingTrip /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openTubingTrip: %f\n", targetP->openTubingTrip);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/8", (sizeof("/30003/0/8") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 335, 1, reg_buf);
        if (result == 0)
        {
            targetP->openTubingReset = reg_buf[0];
            targetP->openTubingReset /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openTubingReset: %f\n", targetP->openTubingReset);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/9", (sizeof("/30003/0/9") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 336, 2, reg_buf);
        if (result == 0)
        {
            targetP->tubingStableTime = reg_buf[0];
            targetP->tubingStableTime <<= 16;
            targetP->tubingStableTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "tubingStableTime: %d\n", targetP->tubingStableTime);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/10", (sizeof("/30003/0/10") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 339, 1, reg_buf);
        if (result == 0)
        {
            if (reg_buf[0] == 1)
            {
                targetP->openTubingEnable = true;
            }
            else
            {
                targetP->openTubingEnable = false;
            }
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openTubingEnable: %d\n", targetP->openTubingEnable);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/11", (sizeof("/30003/0/11") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 264, 1, reg_buf);
        if (result == 0)
        {
            targetP->openCasingTrip = reg_buf[0];
            targetP->openCasingTrip /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingTrip: %f\n", targetP->openCasingTrip);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/12", (sizeof("/30003/0/12") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 265, 1, reg_buf);
        if (result == 0)
        {
            targetP->openCasingReset = reg_buf[0];
            targetP->openCasingReset /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingReset: %f\n", targetP->openCasingReset);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/13", (sizeof("/30003/0/13") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 266, 2, reg_buf);
        if (result == 0)
        {
            targetP->openCasingStableTime = reg_buf[0];
            targetP->openCasingStableTime <<= 16;
            targetP->openCasingStableTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingStableTime: %d\n", targetP->openCasingStableTime);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/14", (sizeof("/30003/0/14") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 285, 1, reg_buf);
        if (result == 0)
        {
            if (reg_buf[0] == 1)
            {
                targetP->openCasingEnable = true;
            }
            else
            {
                targetP->openCasingEnable = false;
            }
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingEnable: %d\n", targetP->openCasingEnable);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/15", (sizeof("/30003/0/15") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 274, 1, reg_buf);
        if (result == 0)
        {
            targetP->openCasingLineDiffTrip = reg_buf[0];
            targetP->openCasingLineDiffTrip /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingLineDiffTrip: %f\n", targetP->openCasingLineDiffTrip);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/16", (sizeof("/30003/0/16") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 275, 1, reg_buf);
        if (result == 0)
        {
            targetP->openCasingLineDiffReset = reg_buf[0];
            targetP->openCasingLineDiffReset /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingLineDiffReset: %f\n", targetP->openCasingLineDiffReset);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/17", (sizeof("/30003/0/17") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 276, 2, reg_buf);
        if (result == 0)
        {
            targetP->openCasingLineDiffStableTime = reg_buf[0];
            targetP->openCasingLineDiffStableTime <<= 16;
            targetP->openCasingLineDiffStableTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingLineDiffStableTime: %d\n", targetP->openCasingLineDiffStableTime);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/18", (sizeof("/30003/0/18") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 286, 1, reg_buf);
        if (result == 0)
        {
            if (reg_buf[0] == 1)
            {
                targetP->openCasingLineDiffEnable = true;
            }
            else
            {
                targetP->openCasingLineDiffEnable = false;
            }
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "openCasingLineDiffEnable: %d\n", targetP->openCasingLineDiffEnable);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/19", (sizeof("/30003/0/19") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 280, 1, reg_buf);
        if (result == 0)
        {
            targetP->closeCasingTrip = reg_buf[0];
            targetP->closeCasingTrip /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "closeCasingTrip: %f\n", targetP->closeCasingTrip);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/20", (sizeof("/30003/0/20") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 281, 1, reg_buf);
        if (result == 0)
        {
            targetP->closeCasingReset = reg_buf[0];
            targetP->closeCasingReset /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "closeCasingReset: %f\n", targetP->closeCasingReset);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/21", (sizeof("/30003/0/21") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 282, 2, reg_buf);
        if (result == 0)
        {
            targetP->closeCasingStableTime = reg_buf[0];
            targetP->closeCasingStableTime <<= 16;
            targetP->closeCasingStableTime += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "closeCasingReset: %d\n", targetP->closeCasingStableTime);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/22", (sizeof("/30003/0/22") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 279, 1, reg_buf);
        if (result == 0)
        {
            targetP->closeCasingPressureThrPerMin = reg_buf[0];
            targetP->closeCasingPressureThrPerMin /= 10;
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "closeCasingPressureThrPerMin: %f\n", targetP->closeCasingPressureThrPerMin);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/23", (sizeof("/30003/0/23") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 149, 2, reg_buf);
        if (result == 0)
        {
            targetP->casingPressureRateTripDelay = reg_buf[0];
            targetP->casingPressureRateTripDelay <<= 16;
            targetP->casingPressureRateTripDelay += reg_buf[1];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "casingPressureRateTripDelay: %d\n", targetP->casingPressureRateTripDelay);
#endif    
    }
    if (lwm2m_stringToUri("/30003/0/24", (sizeof("/30003/0/24") - 1), &uri))
    {
        result = read_holding(FRIENDLY_NAME, modbus_stn_id, 287, 1, reg_buf);
        if (result == 0)
        {
            targetP->closeCPEnable = reg_buf[0];
        }
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "closeCPEnable: %d\n", targetP->closeCPEnable);
#endif    
    }
    if (lwm2m_stringToUri("/30002/0/5701", (sizeof("/30003/0/5701") - 1), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->units, 0x00, 4);
            if (char_buf[0] == 0x00)
            {
                strcpy(targetP->units, "PSI");
            }
            else
            {
                strcpy(targetP->units, "kPa");
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
        *dataArrayP = lwm2m_data_new(25);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 25;
        (*dataArrayP)[0].id = 1;
        lwm2m_data_encode_int(targetP->optimizationType, *dataArrayP + 0);
        (*dataArrayP)[1].id = 2;
        lwm2m_data_encode_int(targetP->arrivalGuardTime, *dataArrayP + 1);
        (*dataArrayP)[2].id = 3;
        lwm2m_data_encode_int(targetP->afterflowScaleFactor, *dataArrayP + 2);
        (*dataArrayP)[3].id = 4;
        lwm2m_data_encode_int(targetP->closeScaleFactor, *dataArrayP + 3);
        (*dataArrayP)[4].id = 5;
        lwm2m_data_encode_int(targetP->adjustmentType, *dataArrayP + 4);
        (*dataArrayP)[5].id = 6;
        lwm2m_data_encode_int(targetP->velocitySource, *dataArrayP + 5);
        (*dataArrayP)[6].id = 7;
        lwm2m_data_encode_float(targetP->openTubingTrip, *dataArrayP + 6);
        (*dataArrayP)[7].id = 8;
        lwm2m_data_encode_float(targetP->openTubingReset, *dataArrayP + 7);
        (*dataArrayP)[8].id = 9;
        lwm2m_data_encode_int(targetP->tubingStableTime, *dataArrayP + 8);
        (*dataArrayP)[9].id = 10; 
        lwm2m_data_encode_bool(targetP->openTubingEnable, *dataArrayP + 9);
        (*dataArrayP)[10].id = 11; 
        lwm2m_data_encode_float(targetP->openCasingTrip, *dataArrayP + 10);   
        (*dataArrayP)[11].id = 12; 
        lwm2m_data_encode_float(targetP->openCasingReset, *dataArrayP + 11);
        (*dataArrayP)[12].id = 13; 
        lwm2m_data_encode_int(targetP->openCasingStableTime, *dataArrayP + 12);
        (*dataArrayP)[13].id = 14; 
        lwm2m_data_encode_bool(targetP->openCasingEnable, *dataArrayP + 13);
        (*dataArrayP)[14].id = 15; 
        lwm2m_data_encode_float(targetP->openCasingLineDiffTrip, *dataArrayP + 14);  
        (*dataArrayP)[15].id = 16; 
        lwm2m_data_encode_float(targetP->openCasingLineDiffReset, *dataArrayP + 15);
        (*dataArrayP)[16].id = 17; 
        lwm2m_data_encode_int(targetP->openCasingLineDiffStableTime, *dataArrayP + 16);
        (*dataArrayP)[17].id = 18; 
        lwm2m_data_encode_bool(targetP->openCasingLineDiffEnable, *dataArrayP + 17);
        (*dataArrayP)[18].id = 19; 
        lwm2m_data_encode_float(targetP->closeCasingTrip, *dataArrayP + 18);        
        (*dataArrayP)[19].id = 20; 
        lwm2m_data_encode_float(targetP->closeCasingReset, *dataArrayP + 19);
        (*dataArrayP)[20].id = 21; 
        lwm2m_data_encode_int(targetP->closeCasingStableTime, *dataArrayP + 20);        
        (*dataArrayP)[21].id = 22; 
        lwm2m_data_encode_float(targetP->closeCasingPressureThrPerMin, *dataArrayP + 21);
        (*dataArrayP)[22].id = 23; 
        lwm2m_data_encode_int(targetP->casingPressureRateTripDelay, *dataArrayP + 22);
        (*dataArrayP)[23].id = 24; 
        lwm2m_data_encode_int(targetP->closeCPEnable, *dataArrayP + 23);        
        (*dataArrayP)[24].id = 5701; // resource 5701
        lwm2m_data_encode_string(targetP->units, *dataArrayP + 24);
    }
    else
    {
        for (i = 0 ; i < *numDataP ; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 201, 1, reg_buf);
                if (result == 0)
                {
                    targetP->optimizationType = reg_buf[0];
                    lwm2m_data_encode_int(targetP->optimizationType, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 2:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 125, 2, reg_buf);
                if (result == 0)
                {
                    targetP->arrivalGuardTime = reg_buf[0];
                    targetP->arrivalGuardTime <<= 16;
                    targetP->arrivalGuardTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->arrivalGuardTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 3:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 202, 1, reg_buf);
                if (result == 0)
                {
                    targetP->afterflowScaleFactor = reg_buf[0];
                    lwm2m_data_encode_int(targetP->afterflowScaleFactor, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 4:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 204, 1, reg_buf);
                if (result == 0)
                {
                    targetP->closeScaleFactor = reg_buf[0];
                    lwm2m_data_encode_int(targetP->closeScaleFactor, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 5:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 206, 1, reg_buf);
                if (result == 0)
                {
                    targetP->adjustmentType = reg_buf[0];
                    lwm2m_data_encode_int(targetP->adjustmentType, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 6:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 200, 1, reg_buf);
                if (result == 0)
                {
                    targetP->velocitySource = reg_buf[0];
                    lwm2m_data_encode_int(targetP->velocitySource, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 7:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 334, 1, reg_buf);
                if (result == 0)
                {
                    targetP->openTubingTrip = reg_buf[0];
                    printf("openTubingTrip float = %f\n", targetP->openTubingTrip);
                    targetP->openTubingTrip /= 10;
                    printf("openTubingTrip divided by 10 = %f\n", targetP->openTubingTrip);
                    lwm2m_data_encode_float(targetP->openTubingTrip, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 8:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 335, 1, reg_buf);
                if (result == 0)
                {
                    targetP->openTubingReset = reg_buf[0];
                    targetP->openTubingReset /= 10;
                    lwm2m_data_encode_float(targetP->openTubingReset, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 9:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 336, 2, reg_buf);
                if (result == 0)
                {
                    targetP->tubingStableTime = reg_buf[0];
                    targetP->tubingStableTime <<= 16;
                    targetP->tubingStableTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->tubingStableTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                 
                break;             
            case 10:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 339, 1, reg_buf);
                if (result == 0)
                {
                    if (reg_buf[0] == 1)
                    {
                        targetP->openTubingEnable = true;
                    }
                    else
                    {
                        targetP->openTubingEnable = false;
                    }
                    lwm2m_data_encode_bool(targetP->openTubingEnable, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;
            case 11:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 264, 1, reg_buf);
                if (result == 0)
                {
                    targetP->openCasingTrip = reg_buf[0];
                    targetP->openCasingTrip /= 10;
                    lwm2m_data_encode_float(targetP->openCasingTrip, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;               
            case 12:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 265, 1, reg_buf);
                if (result == 0)
                {
                    targetP->openCasingReset = reg_buf[0];
                    targetP->openCasingReset /= 10;
                    lwm2m_data_encode_float(targetP->openCasingReset, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;                
            case 13:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 266, 2, reg_buf);
                if (result == 0)
                {
                    targetP->openCasingStableTime = reg_buf[0];
                    targetP->openCasingStableTime <<= 16;
                    targetP->openCasingStableTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->openCasingStableTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;                
            case 14:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 285, 1, reg_buf);
                if (result == 0)
                {
                    if (reg_buf[0] == 1)
                    {
                        targetP->openCasingEnable = true;
                    }
                    else
                    {
                        targetP->openCasingEnable = false;
                    }
                    lwm2m_data_encode_bool(targetP->openCasingEnable, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 15:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 274, 1, reg_buf);
                if (result == 0)
                {
                    targetP->openCasingLineDiffTrip = reg_buf[0];
                    targetP->openCasingLineDiffTrip /= 10;
                    lwm2m_data_encode_float(targetP->openCasingLineDiffTrip, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 16:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 275, 1, reg_buf);
                if (result == 0)
                {
                    targetP->openCasingLineDiffReset = reg_buf[0];
                    targetP->openCasingLineDiffReset /= 10;
                    lwm2m_data_encode_float(targetP->openCasingLineDiffReset, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break; 
            case 17:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 276, 2, reg_buf);
                if (result == 0)
                {
                    targetP->openCasingLineDiffStableTime = reg_buf[0];
                    targetP->openCasingLineDiffStableTime <<= 16;
                    targetP->openCasingLineDiffStableTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->openCasingLineDiffStableTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;
            case 18:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 286, 1, reg_buf);
                if (result == 0)
                {
                    if (reg_buf[0] == 1)
                    {
                        targetP->openCasingLineDiffEnable = true;
                    }
                    else
                    {
                        targetP->openCasingLineDiffEnable = false;
                    }
                    lwm2m_data_encode_bool(targetP->openCasingLineDiffEnable, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                 
                break;   
            case 19:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 280, 1, reg_buf);
                if (result == 0)
                {
                    targetP->closeCasingTrip = reg_buf[0];
                    targetP->closeCasingTrip /= 10;
                    lwm2m_data_encode_float(targetP->closeCasingTrip, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;    
            case 20:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 281, 1, reg_buf);
                if (result == 0)
                {
                    targetP->closeCasingReset = reg_buf[0];
                    targetP->closeCasingReset /= 10;
                    lwm2m_data_encode_float(targetP->closeCasingReset, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break; 
            case 21:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 282, 2, reg_buf);
                if (result == 0)
                {
                    targetP->closeCasingStableTime = reg_buf[0];
                    targetP->closeCasingStableTime <<= 16;
                    targetP->closeCasingStableTime += reg_buf[1];
                    lwm2m_data_encode_int(targetP->closeCasingStableTime, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;   
            case 22:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 279, 1, reg_buf);
                if (result == 0)
                {
                    targetP->closeCasingPressureThrPerMin = reg_buf[0];
                    targetP->closeCasingPressureThrPerMin /= 10;
                    lwm2m_data_encode_float(targetP->closeCasingPressureThrPerMin, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }                
                break;    
            case 23:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 149, 2, reg_buf);
                if (result == 0)
                {
                    targetP->casingPressureRateTripDelay = reg_buf[0];
                    targetP->casingPressureRateTripDelay <<= 16;
                    targetP->casingPressureRateTripDelay += reg_buf[1];
                    lwm2m_data_encode_int(targetP->casingPressureRateTripDelay, *dataArrayP + i);    
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }               
                break;  
            case 24:
                result = read_holding(FRIENDLY_NAME, modbus_stn_id, 287, 1, reg_buf);
                if (result == 0)
                {
                    targetP->closeCPEnable = reg_buf[0];
                    lwm2m_data_encode_int(targetP->closeCPEnable, *dataArrayP + i);    
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
                    memset(targetP->units, 0x00, 4);
                    if (char_buf[0] == 0x00)
                    {
                        strcpy(targetP->units, "PSI");
                    }
                    else
                    {
                        strcpy(targetP->units, "kPa");
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
        *dataArrayP = lwm2m_data_new(25);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 25;
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
        (*dataArrayP)[20].id = 21;
        (*dataArrayP)[21].id = 22;
        (*dataArrayP)[22].id = 23;
        (*dataArrayP)[23].id = 24;
        (*dataArrayP)[24].id = 5701;
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
            case 21:
            case 22:
            case 23:
            case 24:
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
    double temp_float;
    char ascii_float[32];
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 1:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->optimizationType)))
            {
                return COAP_400_BAD_REQUEST;
            }
            reg_buf[0] = targetP->optimizationType;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 201, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 2:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->arrivalGuardTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->arrivalGuardTime >> 16;
            reg_buf[1] = targetP->arrivalGuardTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 125, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 3:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->afterflowScaleFactor)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->afterflowScaleFactor;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 202, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;            
        case 4:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->closeScaleFactor)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->closeScaleFactor;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 204, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;             
        case 5:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->adjustmentType)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->adjustmentType;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 206, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 6:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->velocitySource)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->velocitySource;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 200, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 7:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openTubingTrip)))
            {
                return COAP_400_BAD_REQUEST;
            }
            /* Convert float to int before storing in register as documented format: 1000 = 100.0*/
            temp_float = targetP->openTubingTrip;
            printf("openTubingTrip = %f\n", temp_float);
            temp_float *= 10;
            printf("openTubingTrip multiplied by 10 = %f\n", temp_float);
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            printf("reg_buf[0] = %d\n", reg_buf[0]);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 334, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;
        case 8:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openTubingReset)))
            {
                return COAP_400_BAD_REQUEST;
            }
            temp_float = targetP->openTubingReset;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 335, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;
        case 9:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->tubingStableTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->tubingStableTime >> 16;
            reg_buf[1] = targetP->tubingStableTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 336, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;                
        case 10:
            if (1 != lwm2m_data_decode_bool(dataArray + i, &(targetP->openTubingEnable)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            if (targetP->openTubingEnable == true)
            {
                reg_buf[0] = 1;
            }
            else
            {
                reg_buf[0] = 0;
            }
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 339, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 11:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openCasingTrip)))
            {
                return COAP_400_BAD_REQUEST;
            }
            temp_float = targetP->openCasingTrip;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 264, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;                 
        case 12:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openCasingReset)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            temp_float = targetP->openCasingReset;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 265, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                 
        case 13:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->openCasingStableTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->openCasingStableTime >> 16;
            reg_buf[1] = targetP->openCasingStableTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 266, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }           
            break;                
        case 14:
            if (1 != lwm2m_data_decode_bool(dataArray + i, &(targetP->openCasingEnable)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            if (targetP->openCasingEnable == true)
            {
                reg_buf[0] = 1;
            }
            else 
            {
                reg_buf[0] = 0;
            }
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 285, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }           
            break;                
        case 15:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openCasingLineDiffTrip)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            temp_float = targetP->openCasingLineDiffTrip;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 274, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }           
            break;
        case 16:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openCasingLineDiffReset)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            temp_float = targetP->openCasingLineDiffReset;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 275, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }           
            break;
            break; 
        case 17:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->openCasingLineDiffStableTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->openCasingLineDiffStableTime >> 16;
            reg_buf[1] = targetP->openCasingLineDiffStableTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 276, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 18:
            if (1 != lwm2m_data_decode_bool(dataArray + i, &(targetP->openCasingLineDiffEnable)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            if (targetP->openCasingLineDiffEnable == true)
            {
                reg_buf[0] = 1;
            }
            else
            {
                reg_buf[0] = 0;
            }
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 286, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }
            break;                
        case 19:
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openCasingTrip)))
            {
                return COAP_400_BAD_REQUEST;
            }
            temp_float = targetP->openCasingTrip;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 280, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }             
            break;                
        case 20:  
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->openCasingReset)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            temp_float = targetP->openCasingReset;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 281, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;  
        case 21:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->closeCasingStableTime)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->closeCasingStableTime >> 16;
            reg_buf[1] = targetP->closeCasingStableTime;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 282, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break; 
        case 22:  
            if (1 != lwm2m_data_decode_float(dataArray + i, &(targetP->closeCasingPressureThrPerMin)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            temp_float = targetP->closeCasingPressureThrPerMin;
            temp_float *= 10;
            sprintf(ascii_float, "%.0f", temp_float);
            reg_buf[0] = atoi(ascii_float);
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 279, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 23:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->casingPressureRateTripDelay)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->casingPressureRateTripDelay >> 16;
            reg_buf[1] = targetP->casingPressureRateTripDelay;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 149, reg_buf, 2);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;   
        case 24:
            if (1 != lwm2m_data_decode_int(dataArray + i, &(targetP->closeCPEnable)))
            {
                return COAP_400_BAD_REQUEST;
            } 
            reg_buf[0] = targetP->closeCPEnable;
            result = write_holding(FRIENDLY_NAME, modbus_stn_id, 287, reg_buf, 1);
            if (result != 0)
            {
                return COAP_400_BAD_REQUEST;
            }            
            break;                
        case 25:
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
    case 22:
    case 23:
    case 24:
    case 5701:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30003_object(lwm2m_object_t * object)
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

lwm2m_object_t * get_30003_object(void)
{
    lwm2m_object_t * obj30003;

    obj30003 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30003)
    {
        int i;
        prv_instance_t * targetP;

        memset(obj30003, 0, sizeof(lwm2m_object_t));

        obj30003->objID = PLC_OPT_SAFETY_SETTINGS;
        targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
        if (NULL == targetP) return NULL;
        memset(targetP, 0, sizeof(prv_instance_t));
        targetP->shortID                        = 0;
        targetP->optimizationType               = 0;
        targetP->arrivalGuardTime               = 0;               
        targetP->afterflowScaleFactor           = 0;
        targetP->closeScaleFactor               = 0;
        targetP->adjustmentType                 = 0;
        targetP->velocitySource                 = 0;
        targetP->openTubingTrip                 = 0;
        targetP->openTubingReset                = 0;               
        targetP->tubingStableTime               = 0;
        targetP->openTubingEnable               = false;
        targetP->openCasingTrip                 = 0;
        targetP->openCasingReset                = 0;
        targetP->openCasingStableTime           = 0;
        targetP->openCasingEnable               = false;
        targetP->openCasingLineDiffTrip         = 0;
        targetP->openCasingLineDiffReset        = 0;
        targetP->openCasingLineDiffStableTime   = 0;
        targetP->openCasingLineDiffEnable       = false;
        targetP->closeCasingTrip                = 0;
        targetP->closeCasingReset               = 0;
        targetP->closeCasingStableTime          = 0;               
        targetP->closeCasingPressureThrPerMin   = 0;
        targetP->casingPressureRateTripDelay    = 0;
        targetP->closeCPEnable                  = 0;        
        memset(targetP->units, 0x00, 4);
        obj30003->instanceList = LWM2M_LIST_ADD(obj30003->instanceList, targetP);
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30003->readFunc = prv_read;
        obj30003->discoverFunc = prv_discover;
        obj30003->writeFunc = prv_write;
        obj30003->executeFunc = prv_exec;
        obj30003->createFunc = prv_create;
        obj30003->deleteFunc = prv_delete;
        obj30003->userData = NULL;   // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30003;
}

void free_30003_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

