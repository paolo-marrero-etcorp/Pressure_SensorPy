from api_client import sas_token
from api_client import return_values
from iothub_client import IoTHubClient, IoTHubClientError, IoTHubTransportProvider, IoTHubClientResult
from iothub_client import IoTHubMessage, IoTHubMessageDispositionResult, IoTHubError, DeviceMethodReturnValue
from iothub_client import IoTHubMessageContent

RECEIVE_CALLBACKS = 1
TWIN_CALLBACKS = 1
METHOD_CALLBACKS = 1
SEND_CALLBACKS = 0
HTTP_STATUS_CODE_OK = 200
CONTEXT = 0

def device_method_callback(method_name, payload, context):
    global METHOD_CALLBACKS
    METHOD_CALLBACKS += 1
    print( "Total calls confirmed: %d\n" % METHOD_CALLBACKS )
    return HTTP_STATUS_CODE_OK

def device_twin_callback(update_state, payload, context):
    global TWIN_CALLBACKS
    TWIN_CALLBACKS += 1
    print( "Total twin callbacks confirmed: %d\n" % TWIN_CALLBACKS )

def receive_message_callback(message, context):
    global RECEIVE_CALLBACKS
    RECEIVE_CALLBACKS += 1
    print( "    Total calls received: %d" % RECEIVE_CALLBACKS )
    return IoTHubMessageDispositionResult.ACCEPTED   

def send_confirmation_callback(message, result, user_context):
    global SEND_CALLBACKS
    print ( "Confirmation[%d] received for message with result = %s" % (user_context, result) )
    print ( "    message_id: %s" % message.message_id )
    print ( "    correlation_id: %s" % message.correlation_id )
    SEND_CALLBACKS += 1
    print ( "    Total calls confirmed: %d" % SEND_CALLBACKS )    
    
status, result, connect_string = sas_token.get_connection_string()

if status != return_values.OK:
    print("Connection request has failed. Response: {}".format(return_values.RetValDescription(result)))
    print("Quitting")
    
else:
    print("Connection request success. String: {}".format(connect_string))
    client = IoTHubClient(connect_string.encode("utf-8"), IoTHubTransportProvider.MQTT)
    client.set_option("logtrace", 0)
    client.set_message_callback( receive_message_callback, CONTEXT)        
    client.set_device_twin_callback( device_twin_callback, CONTEXT)
    client.set_device_method_callback( device_method_callback, CONTEXT) 

    message_counter = 0
    while True:
        val = raw_input("Press a key\n")
        if val == 'q':
            break
        else:
            print("Sending message")
            message = IoTHubMessage("Test Message")
            message.message_id = "message_%d" % message_counter
            message.correlation_id = "correlation_%d" % message_counter
            client.send_event_async(message, send_confirmation_callback, message_counter)
            message_counter += 1
        
        
        
        