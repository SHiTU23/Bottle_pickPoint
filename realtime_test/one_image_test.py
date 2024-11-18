import cv2
import numpy as np
import os
import time
import sys

import coordinate_translator

sys.path.append("./")
from keypoint_detection.yolo.bottle_finder import keypoint

image_counter = 0
last_time = 0
record_video = False
FPS = 10
ARUCO_LENGTH = 100
ARUCO_MARKER = cv2.aruco.DICT_5X5_100

translator = coordinate_translator.translator(ARUCO_MARKER, ARUCO_LENGTH)

keypoint_detector = keypoint()

current_dir = os.path.dirname(__file__)
image_path = current_dir + '/3_ArUco.jpg'

image = cv2.imread(image_path)

bottle_data = keypoint_detector.bottle_features(image)
pick_point = tuple(bottle_data[:2])
bottle_orientation = bottle_data[2]
print("bottle key-points: ", bottle_data)
print(f"pick point: {pick_point}")

new_coordinates, new_orientation = translator.translate_coordinate(image, pick_point, bottle_orientation)
new_coordinates = [coords[0] for coords in new_coordinates.tolist()]
new_coordinates = (new_coordinates[:2])
print(new_coordinates[0])
print("new_coordinates: ", new_coordinates, "real_angle: ", new_orientation) ### new_coordinates in mm and angle in degrees

keypoint_detector.show_image_with_keypoints(video_stream=False)