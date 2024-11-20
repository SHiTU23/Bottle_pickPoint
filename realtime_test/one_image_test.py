import cv2
import numpy as np
import os
import time
import sys
import paho.mqtt.client as mqtt
import coordinate_translator

sys.path.append("./")
from keypoint_detection.yolo.bottle_finder import keypoint

image_counter = 0
last_time = 0
record_video = False
FPS = 10

# MQTT Setup
broker_address = "127.0.0.1"  # Localhost, change if needed
broker_port = 1883
x_coord_topic = "coord_X"
y_coord_topic = "coord_Y"
orientation_topic = "Orientation"
bottle_type_topic = "BottleColor"
response_topic = "ResponseToPLC"

mqtt_client = mqtt.Client()
mqtt_client.connect(broker_address, broker_port)

def send_response_message():
    message = "I have found a bottle"
    mqtt_client.publish(response_topic, message)
    print(f"Sent response message: {message}")
    time.sleep(10)

def send_bottle_data(x, y, orientation, color):
    mqtt_client.publish(x_coord_topic, x)
    mqtt_client.publish(y_coord_topic, y)
    mqtt_client.publish(orientation_topic, orientation)
    mqtt_client.publish(bottle_type_topic, color)

    print(f"Data sent:\n  X: {x}\n  Y: {y}\n  Orientation: {orientation}\n  Color: {color}")

# Image Processing Setup
ARUCO_LENGTH = 100
ARUCO_MARKER = cv2.aruco.DICT_5X5_100
translator = coordinate_translator.translator(ARUCO_MARKER, ARUCO_LENGTH)
keypoint_detector = keypoint()

current_dir = os.path.dirname(__file__)
image_path = current_dir + '/3_ArUco.jpg'
image = cv2.imread(image_path)

bottle_data = keypoint_detector.bottle_features(image)
bottle_color = bottle_data[-1]
pick_point = tuple(bottle_data[:2])
bottle_orientation = bottle_data[2]

print("bottle key-points: ", bottle_data)
# print(f"pick point: {pick_point}")
# print(f"bottle_color: {bottle_color}")

aruco = translator.aruco_detector(image)
if aruco != coordinate_translator.NO_ARUCO_FOUND:
    new_coordinates, new_orientation = translator.translate_coordinates(pick_point, bottle_orientation)
    new_coordinates = [coords[0] for coords in new_coordinates.tolist()]
    new_coordinates = (new_coordinates[:2])
    send_response_message()
    print(new_coordinates[0])
    print("new_coordinates: ", new_coordinates, "real_angle: ", new_orientation) ### new_coordinates in mm and angle in degrees

    bottle_x = round(new_coordinates[0])
    bottle_y = round(new_coordinates[1])
    bottle_angle = round(new_orientation)

    # Use the function to send data
    send_bottle_data(bottle_x, bottle_y, bottle_angle, bottle_color)

bottle_data = f"{bottle_x},{bottle_y},{bottle_angle},{bottle_color}"
print(f"bottleData: {bottle_data}")
keypoint_detector.show_image_with_keypoints(video_stream=False)
