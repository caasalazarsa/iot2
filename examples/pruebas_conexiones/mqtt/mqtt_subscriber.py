import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, rc,properties): 
    print("Connected with result  code {0}".format(str(rc)))  
    # Print result of connection attempt
    client.subscribe("digitest/test1")  
    # Subscribe to the topic “digitest/test1”, receive any messages published on it


def on_message(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server. 
    print("Message received-> " + msg.topic + " " + str(msg.payload))  # Print a received msg


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # Create instance of client with client ID “digi_mqtt_test”
client.on_connect = on_connect  # Define callback function for successful connection
client.on_message = on_message  # Define callback function for receipt of a message
# client.connect("m2m.eclipse.org", 1883, 60)  # Connect to (broker, port, keepalive-time)
client.connect('127.0.0.1', 1883)
client.loop_forever()  # Start networking daemon
