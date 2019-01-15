/*------------------------------------------------------------------------------
*
* Description:
*
* Derived from source files in the Eclipse Wakaama project (https://github.com/eclipse/wakaama)
* (c) 2017 - 2018 Extreme Telematics Corp.
*----------------------------------------------------------------------------*/

/*
 * Implements an object for testing purpose
 *
 *                      Multiple
 * Object     |  ID   | Instances | Mandatory |
 *  Pressure  | 3323  |    No     |    No     |
 *
 *  Resources:
 *              Supported    Multiple
 *  Name         | ID   | Operations | Instances | Mandatory |  Type   | Range |         Units                   | Description                                    |
 *  Sensor Value | 5700 |     R      |    No     |    Yes    | Float   |       | Defined by Units resource       | Last or Current Measured Value from the Sensor |
 *  Sensor Units | 5701 |     R      |    No     |    No     | Float   |       |                                 | Measurement Units Definition e.g. Cel for      |
 *               |      |            |           |           |         |       |                                 | Temperature in Celsius                         |
 *  Min Measured | 5601 |     R      |    No     |    No     | Float   |       | Defined by Units resource       | The minimum value measured by the sensor       |
 *  Value        |      |            |           |           |         |       |                                 | since power ON or reset                        |
 *  Max Measured | 5602 |     R      |    No     |    No     | Float   |       | Defined by Units resource       | The maximum value measured by the sensor       |
 *  Value        |      |            |           |           |         |       |                                 | since power ON or reset                        | 
 *  Min Range    | 5603 |     R      |    No     |    No     | Float   |       | Defined by Units resource       | The minimum value that can be measured by the  |
 *  Value        |      |            |           |           |         |       |                                 | sensor                                         |
 *  Min Range    | 5604 |     R      |    No     |    No     | Float   |       | Defined by Units resource       | The maximum value that can be measured by the  |
 *  Value        |      |            |           |           |         |       |                                 | sensor                                         |
 *  Reset Min and| 5605 |     R      |    No     |    No     | Float   |       |                                 | Reset the Min and Max Measured Values to       |
 *  max measured Values |            |           |           |         |       |                                 | Current Value                                  |
 *  Current      | 5821 |     R      |    No     |    No     | Float   |       |                                 | Read or Write the current calibration          |
 *  Calibration  |      |            |           |           |         |       |                                 | coefficient                                    |
 *  Application  | 5750 |     RW     |    No     |    No     | Float   |       |                                 | The application type of the sensor or actuator |
 *  Type         |      |     RW     |           |           |         |       |                                 | coefficient                                    |
 */

#include "liblwm2m.h"
#include "lwm2mclient.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define PRV_TLV_BUFFER_SIZE 64

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
    double   sensorValue;
    double   minRangeValue;
    double   maxRangeValue;
    char     units[4];
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

static uint8_t prv_read(uint16_t instanceId,
                        int * numDataP,
                        lwm2m_data_t ** dataArrayP,
                        lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i;

    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(4);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 4;
        (*dataArrayP)[0].id = 5603;
        (*dataArrayP)[1].id = 5604;
        (*dataArrayP)[2].id = 5700;
        (*dataArrayP)[3].id = 5701;
    }

    for (i = 0 ; i < *numDataP ; i++)
    {
        switch ((*dataArrayP)[i].id)
        {
        case 5603:
            lwm2m_data_encode_float(targetP->minRangeValue, *dataArrayP + i);
            break;
        case 5604:
            lwm2m_data_encode_float(targetP->maxRangeValue, *dataArrayP + i);
            break;
        case 5700:
            if (ai1_millivolts >= 4500)
            {
                targetP->sensorValue = 1000.0;
            }
            else if (ai1_millivolts <= 500)
            {
                targetP->sensorValue = 0;
            }
            else
            {
                targetP->sensorValue = ((ai1_millivolts - 500) / 4000) * 1000;    
            }
            lwm2m_data_encode_float(targetP->sensorValue, *dataArrayP + i);
            break;
        case 5701:
            lwm2m_data_encode_string(targetP->units, *dataArrayP + i);
            break;
        default:
            return COAP_404_NOT_FOUND;
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
        *dataArrayP = lwm2m_data_new(4);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 4;
        (*dataArrayP)[0].id = 5603;
        (*dataArrayP)[1].id = 5604;
        (*dataArrayP)[2].id = 5700;
        (*dataArrayP)[3].id = 5701;
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

    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 5603:
        case 5604:
        case 5700:
        case 5701:
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
    /*
    prv_instance_t * targetP;

    objectP->instanceList = lwm2m_list_remove(objectP->instanceList, id, (lwm2m_list_t **)&targetP);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    lwm2m_free(targetP);

    return COAP_202_DELETED;
    */
    return COAP_405_METHOD_NOT_ALLOWED;
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

    if (NULL == lwm2m_list_find(objectP->instanceList, instanceId)) return COAP_404_NOT_FOUND;

    switch (resourceId)
    {
    case 5603:
    case 5604:
    case 5700:
    case 5701:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_3323_object(lwm2m_object_t * object)
{
#ifdef WITH_LOGS
    fprintf(stdout, "  /%u: Test object, instances:\r\n", object->objID);
    prv_instance_t * instance = (prv_instance_t *)object->instanceList;
    while (instance != NULL)
    {
        fprintf(stdout, "    /%u/%u: shortId: %u, test: %u\r\n",
                object->objID, instance->shortID,
                instance->shortID, instance->test);
        instance = (prv_instance_t *)instance->next;
    }
#endif
}

lwm2m_object_t * get_3323_object(void)
{
    lwm2m_object_t * testObj;

    testObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != testObj)
    {
        int i;
        prv_instance_t * targetP;
        memset(testObj, 0, sizeof(lwm2m_object_t));
        testObj->objID = PRESSURE_3323;
        targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
        if (NULL == targetP) return NULL;
        memset(targetP, 0, sizeof(prv_instance_t));
        targetP->shortID = 0;
        targetP->sensorValue = 0;
        targetP->minRangeValue = 0;
        targetP->maxRangeValue = 1000;
        strncpy(targetP->units, "PSI", sizeof("PSI"));
        testObj->instanceList = LWM2M_LIST_ADD(testObj->instanceList, targetP);
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        testObj->readFunc = prv_read;
        testObj->discoverFunc = prv_discover;
        testObj->writeFunc = prv_write;
        testObj->executeFunc = prv_exec;
        testObj->createFunc = prv_create;
        testObj->deleteFunc = prv_delete;
    }

    return testObj;
}

void free_3323_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

