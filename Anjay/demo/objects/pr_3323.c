/*
 * Copyright 2017-2019 AVSystem <avsystem@avsystem.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../demo.h"
#include "../demo_utils.h"
#include "../objects.h"

// #include "/usr/include/api_client_c/api_client.h"
// #include "/usr/include/api_client_c/analog.h"
// #include "/usr/include/api_client_c/digital.h"
// #include "/usr/include/api_client_c/gps.h"
// #include "/usr/include/api_client_c/conn.h"

#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <avsystem/commons/memory.h>
#include <avsystem/commons/vector.h>

#define SENSOR_VALUE 5700
#define SENSOR_UNITS 5701
#define MIN_MEASURED_VALUE 5601
#define MAX_MEASURED_VALUE 5602
#define MIN_RANGE_VALUE 5603
#define MAX_RANGE_VALUE 5604
#define RESET_MIN_MAX_MEASURED_VALUES 5605
#define CURRENT_CALIBRATION 5821
#define APPLICATION_TYPE 5750

typedef struct {
    int number;
    char *value;
} pressure_exec_arg_t;

typedef struct pressure_instance_struct {
    anjay_iid_t iid;
    float   sensorValue;
    float   minRangeValue;
    float   maxRangeValue;
    float   minMeasuredValue;
    float   maxMeasuredValue;
    char    sensorUnits[128];
    char    currentCalibration[128];
    char    applicationType[128];
    AVS_LIST(pressure_exec_arg_t) last_exec_args;

/*
    int32_t execute_counter;
    bool volatile_res_present;
    int32_t volatile_res_value;
    int32_t bytes_size;
    int32_t bytes_burst;
    bool bytes_zero_begin;
    void *raw_bytes;
    size_t raw_bytes_size;
    AVS_LIST(test_array_entry_t) array;
    AVS_LIST(pressure_exec_arg_t) last_exec_args;
    int32_t test_res_int;
    bool test_res_bool;
    float test_res_float;
    char test_res_string[128];
    struct {
        anjay_oid_t oid;
        anjay_iid_t iid;
    } test_res_objlnk;
*/
} pressure_instance_t;

typedef struct {
    const anjay_dm_object_def_t *def;
    AVS_LIST(struct pressure_instance_struct) instances;
} pressure_repr_t;

static inline pressure_repr_t *
get_pressure(const anjay_dm_object_def_t *const *obj_ptr) {
    assert(obj_ptr);
    return container_of(obj_ptr, pressure_repr_t, def);
}

static pressure_instance_t *find_instance(const pressure_repr_t *repr,
                                      anjay_iid_t iid) {
    AVS_LIST(pressure_instance_t) it;
    AVS_LIST_FOREACH(it, repr->instances) {
        if (it->iid == iid) {
            return it;
        }
    }

    return NULL;
}

static void release_instance(pressure_instance_t *inst) {
    AVS_LIST_CLEAR(&inst->last_exec_args) {
        avs_free(inst->last_exec_args->value);
    }
/*
    avs_free(inst->raw_bytes);
    AVS_LIST_CLEAR(&inst->array);
*/
}

static int test_resource_read(anjay_t *anjay,
                              const anjay_dm_object_def_t *const *obj_ptr,
                              anjay_iid_t iid,
                              anjay_rid_t rid,
                              anjay_output_ctx_t *ctx) {
    (void) anjay;

    pressure_repr_t *test = get_pressure(obj_ptr);
    pressure_instance_t *inst = find_instance(test, iid);
    assert(inst);

    switch (rid) {
    case SENSOR_VALUE:
        return anjay_ret_float(ctx, inst->sensorValue);
    case SENSOR_UNITS:
        return anjay_ret_string(ctx, inst->sensorUnits);
    case MIN_MEASURED_VALUE:
        return anjay_ret_float(ctx, inst->minMeasuredValue);
    case MAX_MEASURED_VALUE:
        return anjay_ret_float(ctx, inst->maxMeasuredValue);
    case MIN_RANGE_VALUE:
        return anjay_ret_float(ctx, inst->minRangeValue);
    case MAX_RANGE_VALUE:
        return anjay_ret_float(ctx, inst->maxRangeValue);
    case CURRENT_CALIBRATION:
        return anjay_ret_string(ctx, inst->currentCalibration);
    case APPLICATION_TYPE:
        return anjay_ret_string(ctx, inst->applicationType);
    default:
        return ANJAY_ERR_NOT_FOUND;
    }
}

static int test_resource_write(anjay_t *anjay,
                               const anjay_dm_object_def_t *const *obj_ptr,
                               anjay_iid_t iid,
                               anjay_rid_t rid,
                               anjay_input_ctx_t *ctx) {
    (void) anjay;

    pressure_repr_t *test = get_pressure(obj_ptr);
    pressure_instance_t *inst = find_instance(test, iid);
    assert(inst);

    switch (rid) {
    case SENSOR_UNITS:
        return anjay_get_string(ctx, inst->sensorUnits, sizeof(inst->sensorUnits));
    case CURRENT_CALIBRATION:
        return anjay_get_string(ctx, inst->currentCalibration, sizeof(inst->currentCalibration));
    case APPLICATION_TYPE:
        return anjay_get_string(ctx, inst->applicationType, sizeof(inst->applicationType));
    default:
        return ANJAY_ERR_NOT_FOUND;
    }
}

static int read_exec_arg_value(anjay_execute_ctx_t *arg_ctx,
                               char **out_string) {
#define VALUE_CHUNK_SIZE 256
    size_t bytes_read = 0;
    ssize_t result = 0;

    while (true) {
        size_t new_value_size = bytes_read + VALUE_CHUNK_SIZE;

        char *new_string = (char *) avs_realloc(*out_string, new_value_size);
        if (!new_string) {
            demo_log(ERROR, "out of memory");
            result = ANJAY_ERR_INTERNAL;
            goto fail;
        }

        *out_string = new_string;

        result = anjay_execute_get_arg_value(arg_ctx, new_string + bytes_read,
                                             (ssize_t) VALUE_CHUNK_SIZE);

        if (result < 0) {
            demo_log(ERROR, "could not read arg value: %d", (int) result);
            goto fail;
        } else if ((size_t) result != VALUE_CHUNK_SIZE - 1) {
            // nothing more to read, we're done
            break;
        }

        // incomplete read; bigger buffer required
        bytes_read += (size_t) result;
    }

    return 0;

fail:
    avs_free(*out_string);
    *out_string = NULL;
    return (int) result;
}

static int read_exec_arg(anjay_execute_ctx_t *arg_ctx,
                         AVS_LIST(pressure_exec_arg_t) *insert_ptr) {
    int arg_number;
    bool has_value;

    int result = anjay_execute_get_next_arg(arg_ctx, &arg_number, &has_value);
    if (result) {
        return result;
    }

    AVS_LIST(pressure_exec_arg_t) arg = AVS_LIST_NEW_ELEMENT(pressure_exec_arg_t);
    if (!arg) {
        demo_log(ERROR, "out of memory");
        return ANJAY_ERR_INTERNAL;
    }

    arg->number = arg_number;

    if (has_value) {
        int result = read_exec_arg_value(arg_ctx, &arg->value);
        if (result) {
            AVS_LIST_DELETE(&arg);
            demo_log(ERROR, "could not get read arg %d value", arg_number);
            return result;
        }
    }

    AVS_LIST_INSERT(insert_ptr, arg);
    return 0;
}

static int read_exec_args(anjay_execute_ctx_t *arg_ctx,
                          AVS_LIST(pressure_exec_arg_t) *out_args) {
    AVS_LIST_CLEAR(out_args) {
        avs_free((*out_args)->value);
    }

    int result;
    AVS_LIST(pressure_exec_arg_t) *tail = out_args;

    while ((result = read_exec_arg(arg_ctx, tail)) == 0) {
        demo_log(DEBUG, "got arg %d", (*tail)->number);
        tail = AVS_LIST_NEXT_PTR(tail);
    }

    if (result == ANJAY_EXECUTE_GET_ARG_END) {
        return 0;
    }

    AVS_LIST_CLEAR(out_args) {
        avs_free((*out_args)->value);
    }
    return result;
}

static int test_resource_execute(anjay_t *anjay,
                                 const anjay_dm_object_def_t *const *obj_ptr,
                                 anjay_iid_t iid,
                                 anjay_rid_t rid,
                                 anjay_execute_ctx_t *arg_ctx) {
    (void) arg_ctx;

    pressure_repr_t *test = get_pressure(obj_ptr);
    pressure_instance_t *inst = find_instance(test, iid);
    assert(inst);

    switch (rid) {
    case SENSOR_UNITS:
    case SENSOR_VALUE:
    case MIN_MEASURED_VALUE:
    case MAX_MEASURED_VALUE:
    case MIN_RANGE_VALUE:
    case MAX_RANGE_VALUE:
    case CURRENT_CALIBRATION:
    case APPLICATION_TYPE:
        return ANJAY_ERR_METHOD_NOT_ALLOWED;
    case RESET_MIN_MAX_MEASURED_VALUES: {
        int result = read_exec_args(arg_ctx, &inst->last_exec_args);
        if (result) {
            demo_log(ERROR, "could not save Execute arguments");
            return result;
        }

        inst->minMeasuredValue = 0;
        anjay_notify_changed(anjay, (*obj_ptr)->oid, iid, MIN_MEASURED_VALUE);
        inst->maxMeasuredValue = 0;
        anjay_notify_changed(anjay, (*obj_ptr)->oid, iid, MAX_MEASURED_VALUE);
        return 0;
    }
    default:
        return ANJAY_ERR_NOT_FOUND;
    }
}

const anjay_dm_object_def_t PRESSURE_OBJECT = {
    .oid = PRESSURE_SENSOR,
    .supported_rids = ANJAY_DM_SUPPORTED_RIDS(MIN_MEASURED_VALUE,
                                              MAX_MEASURED_VALUE,
                                              MIN_RANGE_VALUE,
                                              MAX_RANGE_VALUE,
                                              RESET_MIN_MAX_MEASURED_VALUES,
                                              SENSOR_VALUE,
                                              SENSOR_UNITS,
                                              APPLICATION_TYPE,
                                              CURRENT_CALIBRATION),
    .handlers = {
        .instance_it = anjay_dm_instance_it_SINGLE,
        .instance_present = anjay_dm_instance_present_SINGLE,
        .resource_present = anjay_dm_resource_present_TRUE,
        .resource_read = test_resource_read,
        .resource_write = test_resource_write,
        .resource_execute = test_resource_execute,
        .transaction_begin = anjay_dm_transaction_NOOP,
        .transaction_validate = anjay_dm_transaction_NOOP,
        .transaction_commit = anjay_dm_transaction_NOOP,
        .transaction_rollback = anjay_dm_transaction_NOOP
    }
};

const anjay_dm_object_def_t **pressure_object_create(void) {
    pressure_repr_t *repr = (pressure_repr_t *) avs_calloc(1, sizeof(pressure_repr_t));
    if (!repr) {
        return NULL;
    }
    repr->def = &PRESSURE_OBJECT;
    AVS_LIST(pressure_instance_t) created = AVS_LIST_NEW_ELEMENT(pressure_instance_t);
    if (!created) {
        printf("Error creating object instance\n");
    }
    created->iid = 0;
    created->sensorValue = 0;
    created->minRangeValue = 0;
    created->maxRangeValue = 0;
    created->minMeasuredValue = 0;
    created->maxMeasuredValue = 0;
    strcpy(created->sensorUnits, "Units");
    strcpy(created->currentCalibration, "Calibration");
    strcpy(created->applicationType, "App Type");

    AVS_LIST(pressure_instance_t) *ptr;
    AVS_LIST_FOREACH_PTR(ptr, &repr->instances) {
        if ((*ptr)->iid > created->iid) {
            break;
        }
    }

    AVS_LIST_INSERT(ptr, created);
    return &repr->def;
}

void pressure_object_release(const anjay_dm_object_def_t **def) {
    if (def) {
        pressure_repr_t *repr = get_pressure(def);
        AVS_LIST_CLEAR(&repr->instances) {
            release_instance(repr->instances);
        }
        avs_free(repr);
    }
}

void pressure_notify_time_dependent(anjay_t *anjay,
                                const anjay_dm_object_def_t **def) {
    pressure_repr_t *repr = get_pressure(def);
    struct pressure_instance_struct *it;
/*    AVS_LIST_FOREACH(it, repr->instances) {
        anjay_notify_changed(anjay, (*def)->oid, it->iid, TEST_RES_TIMESTAMP);
    } */
}
/*
void pressure_api_init(void)
{
    // This creates the friendly names for the hardware to use in the Morpheus App Builder.
    // sleep(10);
    ApiInit();
    int senpwr_handle = sen_power_open("senpwr");	// From Friendly Name from Morpheus App Builder
    int ai1_handle = analog_open("ai1");	// From Friendly Name from Morpheus App Builder
    
    sen_power_set(senpwr_handle);
    if ((senpwr_handle && ai1_handle) == false)
    {
        printf("At least one of analog or sensor power handles is/are invalid\n");
    }
	
    
	// Analog to Digital settle time and test read 
    //  usleep(10000);
    ai1_counts = read_analog_counts(ai1_handle);
    ai1_millivolts = read_analog_millivolts(ai1_handle);
 }
 */
