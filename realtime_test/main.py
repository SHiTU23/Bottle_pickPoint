from pypylon import pylon
import cv2
import numpy as np
import os
import time
import sys

import coordinate_translator
from mqtt_send_data import mqtt_communication

sys.path.append("./")
from keypoint_detection.yolo.bottle_finder import keypoint

class bottle_finder:
    MANUAL_MODE = 'manual'
    AUTO_MODE = 'auto'
    MQTT_CONNECTED = False
    
    
    def __init__(self, aruco_marker, aruco_marker_length, run_mode=MANUAL_MODE):
        '''
            options for run_mode : bottle_finder.MANUAL_MODE / bottle_finder.AUTO_MODE
        '''
        self.current_dir = os.path.dirname(__file__)
        self.savePath = self.current_dir + '/predicted_images' 

        self.run_mode = run_mode
        self.stream = True
        self.FPS = 10
        self.ARUCO_LENGTH = aruco_marker_length
        self.ARUCO_MARKER = aruco_marker

        self.arcuo_translator = coordinate_translator.translator(self.ARUCO_MARKER, self.ARUCO_LENGTH)
        self.keypoint_detector = keypoint()

        self.cap = self.open_camera()
        
        # Start grabbing continuously (camera will start to capture images)
        self.cap.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        
        self.frame_width = (self.cap.Width.Value)//4 ### 612
        self.frame_height = (self.cap.Height.Value)//4 ### 512
        
        if self.run_mode == self.AUTO_MODE:
            self.mqtt_startConnection()
            self.MQTT_CONNECTED = True
        
        if self.stream == True:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 format
            self.out = cv2.VideoWriter(f'{self.current_dir}/videos/prediction_video.mp4', fourcc, self.FPS, (self.frame_width, self.frame_height))

    #################################################
    ####             MQTT Communocation          ####        
    #################################################
            
    def mqtt_startConnection(self):
        self.mqtt_connection = mqtt_communication()
        print("CONNECTED TO MQTT")

    #################################################


    def open_camera(self):
        # Create an instant camera object with the first camera device found
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        return camera
    
    def scan_the_scene(self, pick_range, delay=0, coordinate_offset=(0,0)):
        stream_video_start = False

        bottle_available = False
        notify_mqtt = False
        massage_sent = False
        bottle_is_detected = False
        
        # Convert images to OpenCV format and display
        converter = pylon.ImageFormatConverter()
        # Convert to OpenCV BGR format
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        image_counter = 0
        last_time = 0
        
        counter = 0
        
        while self.cap.IsGrabbing():
            grabResult = self.cap.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            new_time = time.time() 
            if grabResult.GrabSucceeded():
                                    
                # Access the image data as a NumPy array
                image = converter.Convert(grabResult)
                image = image.GetArray()
                image = cv2.resize(image, (self.frame_width, self.frame_height))
                
                ### check for bottles every 0.5 sec if the delay = 0.5
                if new_time - last_time > delay:
                    if self.bottle_is_found(image):
                        bottle_features = self.keypoint_detector.bottle_features(image)
                        if self.aruco_is_visible(image):
                            if bottle_features != None:
                                bottle_pickPose_x, bottle_pickPose_y, bottle_angle, bottle_color = self.bottlePose_in_arucoCoords(bottle_features, coordinate_offset)
                                cv2.putText(self.keypoint_detector._image, (f"real_pick_coords: ({int(bottle_pickPose_x)}, {int(bottle_pickPose_y)})"), (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                                self.keypoint_detector.show_image_with_keypoints()

                                ### if the bottle in the visible-range -> bottle found
                                if -150 <= bottle_pickPose_y < -100: #and (bottle_is_detected == False):
                                    ### notify the PLC to get prepared for stopping the conveyor
                                    print(f"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Bottle is detected ++")
                                    #bottle_is_detected = True
                                    self.mqtt_connection.send_response_message("TRUE","FALSE")

                                elif self.bottle_is_inPickRange(pick_range, bottle_pickPose_y):
                                    print("########################################################### BOTTLE IN PICK RANGE ##")
                                    bottle_available = True
                                    notify_mqtt = True
                                    print(f"in if notify_mqtt is {notify_mqtt}")

                                elif self.bottle_is_inPlaceRange(bottle_pickPose_y):
                                    massage_sent = False
                                    print(f"in if massage_sent is {massage_sent}")

                                elif bottle_pickPose_y < 150 and bottle_pickPose_y > 400: ## bottle out of visible range
                                    bottle_available = False
                                    bottle_is_detected = False
                                    self.mqtt_connection.send_response_message("FALSE","FALSE") 

                                if bottle_available:
                                    ### AUTO mode: communication with MQTT
                                    if self.run_mode == self.AUTO_MODE:
                                        ### send : "I have found a bottle" to mqtt
                                        if notify_mqtt and (massage_sent == False):
                                            print("----------------------------------- send notification ")
                                            counter += 1
                                            notify_mqtt = False
                                            massage_sent = True
                                               
                                        ### keep sending the data
                                        print("---------------------------------------------------------- SEND DATA -----")
                                        #self.mqtt_connection.send_bottle_data(bottle_pickPose_x, bottle_pickPose_y, bottle_angle, bottle_color)
                                        self.mqtt_connection.send_bottle_data(bottle_pickPose_x, bottle_pickPose_y, bottle_angle, bottle_color)
                                        self.mqtt_connection.send_response_message("TRUE","TRUE") 
                                        cv2.putText(self.keypoint_detector._image, (f"BOTTLE IN POSE; Data is Densing"), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 250), 1)
                                        
                                    ### MANUAL mode: NO communcation with MQTT
                                    elif self.run_mode == self.MANUAL_MODE:
                                        print(f"(x, y, theta, color): ({bottle_pickPose_x}, {bottle_pickPose_y}, {bottle_angle}, {bottle_color})") ### new_coordinates in mm and angle in degrees
                                        cv2.putText(self.keypoint_detector._image, (f"BOTTLE IN POSE"), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 250), 1)
                            else:
                                self.mqtt_connection.send_response_message("FALSE","FALSE") 
       
                        else:
                            print("NO ARUCO DETECTED OR NO BOTTLE")

                    last_time = new_time

                    self.keypoint_detector.show_image_with_keypoints()

                if stream_video_start == True:
                    print("recording ...........")
                    self.out.write(image)

                key = cv2.waitKey(1) & 0xFF
                # Break loop if 'ESC' is pressed
                if key == 27 or key == ord('q'):
                    break
                elif key == ord('s'):
                    image_counter += 1
                    cv2.imwrite(f'{self.savePath}/image_{image_counter}.jpg', self.keypoint_detector._image)
                    
                    print(f"********** Image {image_counter} Saved **********")
                elif key == ord('r'):
                    print("record started")
                    stream_video_start = True
            else:
                print("NO FRAME CAPTURED")
        
        grabResult.Release()
        self.cap.StopGrabbing()
        self.cap.Close()
        self.out.release()
        cv2.destroyAllWindows()

    def aruco_is_visible(self, image):
        ###n check for Aruco
        aruco_state = self.arcuo_translator.aruco_detector(image)
        if aruco_state != coordinate_translator.NO_ARUCO_FOUND:
            return True
        else:
            return False

    def bottle_is_found(self, image):
        NO_BOTTLE_FOUND = None
        ### get the bottle features (pick-point, orientation, color)
        bottle_detection = self.keypoint_detector.bottle_detected(image)
        if bottle_detection != NO_BOTTLE_FOUND:
            return True
        else:
            return False
        
    def bottlePose_in_arucoCoords(self, bottle_features, coordinate_offset):
        X_OFFSET, Y_OFFSET = coordinate_offset
        pick_point = tuple(bottle_features[:2])  ### x, y
        bottle_orientation = bottle_features[2]  ### angle
        bottle_color = bottle_features[-1]       ### color

        print("bottle key-points: ", bottle_features)
        print(f"pick point: {pick_point}")

        #### Convert coords to Aruco coordinate sys
        new_coordinates, new_orientation = self.arcuo_translator.translate_coordinates(pick_point, bottle_orientation)
        new_coordinates = [coords[0] for coords in new_coordinates.tolist()]
        new_x, new_y = (new_coordinates[:2]) ### x, y

        bottle_x = round(new_x - X_OFFSET)
        bottle_y = round(new_y - Y_OFFSET)
        bottle_angle = round(new_orientation)

        return bottle_x, bottle_y, bottle_angle, bottle_color
    
    def bottle_is_inPickRange(self, pick_range, pick_point_Y):
        lower_band, upper_band = pick_range
        if lower_band <= pick_point_Y <= upper_band:
            return True
        else:
            return False
        
    def bottle_is_inPlaceRange(self, pick_point_Y):
        ### the range that the bottle might be placed
        place_range = (100, 200)
        lower_band, upper_band = place_range
        if lower_band <= pick_point_Y <= upper_band:
            return True
        else:
            return False


if __name__ == '__main__':
    FPS = 10
    ARUCO_LENGTH = 100
    ARUCO_MARKER = cv2.aruco.DICT_5X5_100

    PICK_RANGE = (10, 300) ### range of Y axis in the Aruco coordinates
    ### these have been found by avaluating by measuring in the real layout
    X_OFFSET = 20
    Y_OFFSET = 20

    bottle_scanner = bottle_finder(ARUCO_MARKER, ARUCO_LENGTH, run_mode=bottle_finder.AUTO_MODE)
    bottle_scanner.scan_the_scene(PICK_RANGE, delay=0, coordinate_offset=(X_OFFSET, Y_OFFSET))



"""
def bottle_is_inPickRange(pick_range, pick_point_Y):
    lower_band, upper_band = pick_range
    if lower_band <= pick_point_Y <= upper_band:
        return True
    else:
        return False

run_mode = 'auto_mode'

NO_BOTTLE_FOUND = None
bottle_available = False

image_counter = 0
last_time = 0
record_video = False
FPS = 10
ARUCO_LENGTH = 100
ARUCO_MARKER = cv2.aruco.DICT_5X5_100

### these have been found by avaluating by measuring in the real layout
X_OFFSET = 20
Y_OFFSET = 20
PICK_RANGE = (400, 550) ### range of Y axis in the Aruco coordinates

current_dir = os.path.dirname(__file__)
savePath = current_dir + '/predicted_images' 

# mqtt_connection = mqtt_communication()

translator = coordinate_translator.translator(ARUCO_MARKER, ARUCO_LENGTH)

keypoint_detector = keypoint()

# Convert images to OpenCV format and display
converter = pylon.ImageFormatConverter()
# Convert to OpenCV BGR format
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Create an instant camera object with the first camera device found
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Start grabbing continuously (camera will start to capture images)
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

frame_width = (camera.Width.Value)//4 ### 612
frame_height = (camera.Height.Value)//4 ### 512\

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 format
out = cv2.VideoWriter(f'{current_dir}/videos/prediction_video.mp4', fourcc, FPS, (frame_width, frame_height))

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    new_time = time.time() 
    if grabResult.GrabSucceeded():
        # Access the image data as a NumPy array
        image = converter.Convert(grabResult)
        image = image.GetArray()
        image = cv2.resize(image, (frame_width, frame_height))

        ### check for bottles every 0.5 sec
        if new_time - last_time > 0.5:
            ###n check for Aruco
            aruco_state = translator.aruco_detector(image)

            ### get the bottle features (pick-point, orientation, color)
            bottle_data = keypoint_detector.bottle_features(image)

            if bottle_data != NO_BOTTLE_FOUND and aruco_state != coordinate_translator.NO_ARUCO_FOUND:
                pick_point = tuple(bottle_data[:2])  ### x, y
                bottle_orientation = bottle_data[2]  ### angle
                bottle_color = bottle_data[-1]       ### color

                print("bottle key-points: ", bottle_data)
                print(f"pick point: {pick_point}")

                #### Convert coords to Aruco coordinate sys
                new_coordinates, new_orientation = translator.translate_coordinates(pick_point, bottle_orientation)
                new_coordinates = [coords[0] for coords in new_coordinates.tolist()]
                new_coordinates = (new_coordinates[:2]) ### x, y

                bottle_x = round(new_coordinates[0] - X_OFFSET)
                bottle_y = round(new_coordinates[1] - Y_OFFSET)
                bottle_angle = round(new_orientation)
            
                print(f"new_coordinates: ({bottle_x},{bottle_y});", "real_angle: ", bottle_angle) ### new_coordinates in mm and angle in degrees
                cv2.putText(image, (f"real_pick_coords: ({int(bottle_x)}, {int(bottle_y)})"), (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
                
                
                ##################################################
                ####             SEND DATA - MQTT             ####
                ##################################################

                ### if the bottle in the pick-range -> bottle found
                if bottle_is_inPickRange(PICK_RANGE, bottle_y):
                    print("***************** BOTTLE IN PICK RANGE *****************")
                    bottle_available = True


            
            last_time = new_time

        # translator.show_aruco()
        keypoint_detector.show_image_with_keypoints()
        # cv2.imshow("image", image)
        if record_video == True:
            print("recording ...........")
            out.write(keypoint_detector._image)

        key = cv2.waitKey(1) & 0xFF
        # Break loop if 'ESC' is pressed
        if key == 27 or key == ord('q'):
            break
        elif key == ord('s'):
            image_counter += 1
            cv2.imwrite(f'{savePath}/image_{image_counter}.jpg', keypoint_detector._image)
            
            print(f"********** Image {image_counter} Saved **********")
        elif key == ord('r'):
            print("record started")
            record_video = True
    
    grabResult.Release()

camera.StopGrabbing()
camera.Close()
out.release()
cv2.destroyAllWindows()

"""