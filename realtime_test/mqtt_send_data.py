import paho.mqtt.client as mqtt
import time

class mqtt_communication:
    # MQTT Setup
    broker_address = "192.168.125.201"  # Localhost, change if needed
    broker_port = 1883
    x_coord_topic = "coord_X"
    y_coord_topic = "coord_Y"
    orientation_topic = "Orientation"
    bottle_type_topic = "BottleColor"
    response_topic = "ResponseToPLC"
    bottleTrueTopicSub = "BottleTrue"
    bottlePickPosTopicSub = "BottlePickPos"


    def __init__(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.broker_address, self.broker_port)

    def send_response_message(self,message,bottle_pickPos):
        self.mqtt_client.publish(self.bottleTrueTopicSub, message)
        self.mqtt_client.publish(self.bottlePickPosTopicSub, bottle_pickPos)
        print(f"Sent response message: {message}")

    def send_bottle_data(self, x, y, orientation, color):
        self.mqtt_client.publish(self.x_coord_topic, x)
        self.mqtt_client.publish(self.y_coord_topic, y)
        self.mqtt_client.publish(self.orientation_topic, orientation)
        self.mqtt_client.publish(self.bottle_type_topic, color)
        print(f"Data sent:\n  X: {x}\n  Y: {y}\n  Orientation: {orientation}\n  Color: {color}\n")