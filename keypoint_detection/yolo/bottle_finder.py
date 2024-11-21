from ultralytics import YOLO
import cv2
import os
import math
import numpy as np

class keypoint:
    def __init__(self):
        _current_dir = os.path.dirname(__file__)
        _model_path = _current_dir + '/best.pt'
        self.model = YOLO(_model_path)

        self.image_path = ''
        self._bottle_orientation = -1 ## no bottle orientation is done
        self.cap_color = '' ### no color detection is done
        self.bottle_keypoints = None

    def predict_keyPoints(self, image_array) -> dict:
        '''
            Prameters:
            image_array : array of image

            Returns: 
            dict: {'cap_corner':0, 'pick_point':0, 'angle_point':0}\n
                  cap_corner: coordinates of bottle's cap top-left corner (for color detection), \n
                  pick_point: coordinates of center point under the cap (pick point), \n
                  angle_point: coordinates of center point at the end of the bottle(for angle caculation)
        '''
        self._image = image_array
        self._image_width, self._image_height = self._image.shape[0], self._image.shape[1]

        _points = []
        self.bottle_keypoints = {'cap_corner':0, 'pick_point':0, 'angle_point':0}

        _results = self.model(self._image)[0]

        ### bottle found
        if len(_results) > 0:
            for result in _results:
                for key_points in (result.keypoints.xy.tolist()):
                    _points.append(key_points)
            self.bottle_keypoints['cap_corner'] = _points[0][0]
            self.bottle_keypoints['pick_point'] = _points[0][1]
            self.bottle_keypoints['angle_point'] = _points[0][2]

            return self.bottle_keypoints
        ### No bottle found
        else: 
            self.bottle_keypoints = None
            return self.bottle_keypoints
    
    def bottle_detected(self, image_array):
        predicted_keyPoints = self.predict_keyPoints(image_array)
        if predicted_keyPoints != None:
            return True
        else:
            return False

    def bottle_features(self, image_array) -> list:
        """
            Returns:
            [x, y, orientation, color]
        """
        predicted_keyPoints = self.predict_keyPoints(image_array)
        if predicted_keyPoints != None:
            pick_point_x, pick_point_y = predicted_keyPoints['pick_point']
            if 0 <= pick_point_x < self._image_width and 0 <= pick_point_y < self._image_height:
                bottle_orientation = self.bottle_orientation()
                bottle_color = self.bottle_color()
                
                bottle_data = [pick_point_x, pick_point_y, bottle_orientation, bottle_color]
                return bottle_data
        else:
            return None
    
    def bottle_color(self) -> str:
        """
            Takes the cap point, creates a bounding box utilizing pick-point. 
            Get average on the pixel color
        """

        if self.bottle_keypoints != None:  ### bottle detected

            ######################################################################################
            ####    Create a point in front of top left corner on the cap, on the right side   ###
            ######################################################################################

            topLeft_x, topLeft_y = self.bottle_keypoints['cap_corner'][0], self.bottle_keypoints['cap_corner'][1]   
            pickPoint_x, pickPoint_y = self.bottle_keypoints['pick_point'][0], self.bottle_keypoints['pick_point'][1]  

            if 0 <= pickPoint_x < self._image_width and 0 <= pickPoint_y < self._image_height:
            
                ### amplitude of the line between pick-point and topleft-corner on the cap
                amplitude = math.sqrt((pickPoint_x - topLeft_x)**2 + (pickPoint_y - topLeft_y)**2) 

                ### the angle of the line
                theta = math.atan2((pickPoint_y-topLeft_y), (pickPoint_x-topLeft_x))
                theta_degrees = theta * (180/np.pi)

                ### the point on the top right corner of the cap
                rotated_angle = (theta_degrees - 120) * (np.pi / 180) ## convert to Raduian

                topRight_x = pickPoint_x + amplitude * math.cos(rotated_angle)
                topRight_y = pickPoint_y + amplitude * math.sin(rotated_angle)

                ############################################################################################################
                ###  find the color of pixels in the triangle created from (pick-point, topLeft-corner, topRight-corner) ###
                ############################################################################################################

                vector1 = (int(topLeft_x), int(topLeft_y))
                vector2 = (int(pickPoint_x), int(pickPoint_y))
                vector3 = (int(topRight_x), int(topRight_y))

                min_x = min(vector1[0], vector2[0], vector3[0])
                max_x = max(vector1[0], vector2[0], vector3[0])

                min_y = min(vector1[1], vector2[1], vector3[1])
                max_y = max(vector1[1], vector2[1], vector3[1])

                _pixel_colors = []
                for x in range(min_x, max_x+1): 
                    for y in range(min_y, max_y+1):
                        x = int(x)
                        y = int(y)
                        if 0 <= x < self._image_height and 0 <= y < self._image_width and self._is_point_in_triangle(x, y, vector1, vector2, vector3):
                            color= self._image[y, x]
                            _pixel_colors.append(color)

                self.cap_color = self._color_majortity(_pixel_colors)
                return self.cap_color
        else: ### no bottle detected 
            self.cap_color = ''
            return self.cap_color 
    def bottle_orientation(self) -> float:
        """
            Calculate the angle using pick-point and angle-point
            if negative, add 360; all angles are between 0 - 360
        """
        if self.bottle_keypoints != None:    
            anglePoint_x, anglePoint_y = self.bottle_keypoints['angle_point'][0], self.bottle_keypoints['angle_point'][1]   
            pickPoint_x, pickPoint_y = self.bottle_keypoints['pick_point'][0], self.bottle_keypoints['pick_point'][1]  

            if 0 <= pickPoint_x < self._image_width and 0 <= pickPoint_y < self._image_height:
                    
                ### the angle of the line
                _bottle_angle = math.atan2((anglePoint_y-pickPoint_y), (anglePoint_x-pickPoint_x))
                _bottle_angle_degrees = _bottle_angle * (180/np.pi)

                self._bottle_orientation = 180 - _bottle_angle_degrees

                if self._bottle_orientation == 360:
                    self._bottle_orientation = 0

                return self._bottle_orientation
        else: ### No bottle found
            self._bottle_orientation = -1
            return self._bottle_orientation

    def show_image_with_keypoints(self, video_stream=True):
        """
            video_stream = True (default) : while real-time images from camera or video are being detected
            video_stream = False : if only one picture is being detected and shown
        """
        # self._image = cv2.imread(self.image)
        if self.bottle_keypoints != None:
            for keypoint in self.bottle_keypoints.items():
                ### put the key points on the image
                keypoint_id = keypoint[0]
                x, y = int(keypoint[1][0]), int(keypoint[1][1])
                cv2.putText(self._image, f"{keypoint_id}: ({x},{y})", (x, y+2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.circle(self._image, (x, y), 2, (0, 0, 255), 3)
                
                if self.cap_color != '': ### color is detected
                    cv2.putText(self._image, f"bottle_color: {self.cap_color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                if self._bottle_orientation != -1: ### orientation is calculated
                    cv2.putText(self._image, f'bottle_orientation: {int(self._bottle_orientation)}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        cv2.imshow("predicted image", self._image)
        if video_stream == False:
            cv2.waitKey(0)

    def save_predicted_image(self, save_path, image_name):
        cv2.imwrite(f"{save_path}/{image_name}.jpg", self._image)

    def _is_point_in_triangle(self, x, y, v1, v2, v3):
    # Using barycentric coordinates method to check if (x, y) lies inside the triangle
        def sign(p1, p2, p3):
            # Calculates the signed area of the triangle formed by p1, p2, and p3
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

        # all of these should have the same sign
        b1 = sign((x, y), v1, v2) < 0.0
        b2 = sign((x, y), v2, v3) < 0.0
        b3 = sign((x, y), v3, v1) < 0.0

        # Check if all signs are the same (either all True or all False)
        return (b1 == b2) and (b2 == b3)

    def _color_detection(self, pixel_color):
        blue, green, red = pixel_color

        if red > blue and green > blue:
            return 'yellow'
        elif red > blue and red > green:
            return 'red'
        elif blue > red and blue > green:
            return 'blue'

    def _color_majortity(self, pixels_color_list):
        colors_counters = {'blue' : 0, 'yellow' : 0, 'red' : 0}
        
        for pixel in pixels_color_list:
            color = self._color_detection(pixel)
            if color == 'blue':
                colors_counters['blue'] += 1
            elif color == 'yellow':
                colors_counters['yellow'] += 1
            elif color == 'red':
                colors_counters['red'] += 1

        return max(colors_counters, key=colors_counters.get)




'''
************************
TO DO
1. test what is the output when there is no bottle in the image ----- DONE

3. test the whole class    ----- DONE

2. test for the camera images - real time
4. test the show funtion when using the camer - the waitkey part might be a problem

'''
if __name__ == "__main__":
    # image_name = 'emptyscene.jpg'
    # image_name = '67_red.jpg_0_1958.jpg'    ## head  up  to the right - red
    # image_name = '36_Videoimages.jpg'       ## head  up  to the left - red
    # image_name = '70_blue.jpg_0_9550.jpg'   ## head down to the right - blue
    image_name = '4_yellow.jpg_0_6674.jpg'  ## head down to the right - yellow
    # image_name = 'yellow_Videoimages.jpg'   ## head up to the right - yellow
    # image_name = '4_yellow.jpg_0_4343.jpg'   ## head down to the left - yellow
    # image_name = '62_Videoimages.jpg'   ## horizontal head to the left - blue

    current_dir = os.path.dirname(__file__)
    image_path = current_dir + '/test_images/' + image_name
    save_path = current_dir + '/predicted_images'

    key_point_detector = keypoint()

    image = cv2.imread(image_path)

    bottle_data = key_point_detector.bottle_features(image)
    print("bottle key-points: ", bottle_data)

    '''
    result = key_point_detector.predict_keyPoints(image)
    print(f"result is : {result}")
    
    bottle_color = key_point_detector.bottle_color()
    print(f"color of the bottle: {bottle_color}")

    bottle_angle = key_point_detector.bottle_orientation()
    print(f"bottle_angle: {bottle_angle}")
    '''

    key_point_detector.show_image_with_keypoints(video_stream=False)