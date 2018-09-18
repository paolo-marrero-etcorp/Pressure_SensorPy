#include "Lwm2m_Lib.h"
#include "liblwm2m.h"
#include <assert.h>
#include <stdio.h>

int main(int argc, char *argv[])
{ 
	lwm2m_context_t * lwm2mH = NULL;
	 
	// malloc test
	lwm2m_object_t * deviceObj;

	deviceObj = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));
	
	assert(deviceObj != NULL);
	
	lwm2m_free(deviceObj);
	
	// tlv parsing test
	// Resource 55 {1, 2, 3}
	uint8_t data1[] = { 0xC3, 55, 1, 2, 3 };
	// Instance 0x203 {Resource 55 {1, 2, 3}, Resource 66 {4, 5, 6, 7, 8, 9, 10, 11, 12 } }
	uint8_t data2[] = { 0x28, 2, 3, 17, 0xC3, 55, 1, 2, 3, 0xC8, 66, 9, 4, 5, 6, 7, 8, 9, 10, 11, 12, };
	// Instance 11 {MultiResource 11 {ResourceInstance 0 {1, 2, 3}, ResourceInstance 1 {4, 5, 6, 7, 8, 9, ... } }
	uint8_t data3[174] = { 0x08, 11, 171, 0x88, 77, 168, 0x43, 0, 1, 2, 3, 0x48, 1, 160, 4, 5, 6, 7, 8, 9 };
	int result;
	lwm2m_data_t *dataP;
	lwm2m_data_t *tlvSubP;

	result = lwm2m_data_parse(NULL, data1, sizeof(data1), LWM2M_CONTENT_TLV, &dataP);
	assert(result == 1);
	assert(dataP != NULL);
	assert(dataP->type == LWM2M_TYPE_OPAQUE);
	assert(dataP->id == 55);
	assert(dataP->value.asBuffer.length == 3);
	assert(0 == memcmp(dataP->value.asBuffer.buffer, &data1[2], 3));
	lwm2m_data_free(result, dataP);

	result = lwm2m_data_parse(NULL, data2, sizeof(data2), LWM2M_CONTENT_TLV, &dataP);
	assert(result = 1);
	
	assert(dataP->type == LWM2M_TYPE_OBJECT_INSTANCE);
	assert(dataP->id == 0x203);
	assert(dataP->value.asChildren.count = 2);
	assert(dataP->value.asChildren.array != NULL);
	tlvSubP = dataP->value.asChildren.array;

	assert(tlvSubP[0].type == LWM2M_TYPE_OPAQUE);
	assert(tlvSubP[0].id == 55);
	assert(tlvSubP[0].value.asBuffer.length == 3);
	assert(0 == memcmp(tlvSubP[0].value.asBuffer.buffer, &data2[6], 3));

	assert(tlvSubP[1].type == LWM2M_TYPE_OPAQUE);
	assert(tlvSubP[1].id == 66);
	assert(tlvSubP[1].value.asBuffer.length == 9);
	assert(0 == memcmp(tlvSubP[1].value.asBuffer.buffer, &data2[12], 9));
	lwm2m_data_free(result, dataP);

	result = lwm2m_data_parse(NULL, data3, sizeof(data3), LWM2M_CONTENT_TLV, &dataP);
	assert(result == 1);
	assert(dataP != NULL);
	assert(dataP->type == LWM2M_TYPE_OBJECT_INSTANCE);
	assert(dataP->id == 11);
	assert(dataP->value.asChildren.count == 1);
	assert(dataP->value.asChildren.array != NULL);
	tlvSubP = dataP->value.asChildren.array;

	assert(tlvSubP[0].type == LWM2M_TYPE_MULTIPLE_RESOURCE);
	assert(tlvSubP[0].id == 77);
	assert(tlvSubP[0].value.asChildren.count == 2);
	assert(tlvSubP[0].value.asChildren.array != NULL);
	tlvSubP = tlvSubP[0].value.asChildren.array;

	assert(tlvSubP[0].type == LWM2M_TYPE_OPAQUE);
	assert(tlvSubP[0].id == 0);
	assert(tlvSubP[0].value.asBuffer.length == 3);
	assert(0 == memcmp(tlvSubP[0].value.asBuffer.buffer, &data3[8], 3));

	assert(tlvSubP[1].type == LWM2M_TYPE_OPAQUE);
	assert(tlvSubP[1].id == 1);
	assert(tlvSubP[1].value.asBuffer.length == 160);
	assert(0 == memcmp(tlvSubP[1].value.asBuffer.buffer, &data3[14], 160));
	lwm2m_data_free(result, dataP);
	
	
	lwm2m_uri_t * uriP;
	int size = 0;
	result = lwm2m_data_parse(NULL, data1, sizeof(data1), LWM2M_CONTENT_TLV, &dataP);
	uint8_t * bufferP;
	
	lwm2m_discover_serialize( NULL, uriP, NULL, size, dataP, &bufferP);
	
}