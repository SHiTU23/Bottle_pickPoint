import cv2
import numpy as np
import os

NO_ARUCO_FOUND = None

class translator:
    def __init__(self, aruco_dictionary, aruco_length):
        """
            Parameter:
                aruco_dictionary : ex: cv2.aruco.DICT_5X5_100
                aruco_length : in mm
        """
        self.aruco_area = 4 * aruco_length
        self.aruco_marker = aruco_dictionary
        self._aruco_corners = NO_ARUCO_FOUND
    
    def aruco_detector(self, image):
        self._image = image
        self._aruco_corners = []
        _parameters = cv2.aruco.DetectorParameters()
        _aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_marker)
        _aruco_detector = cv2.aruco.ArucoDetector(_aruco_dict, _parameters)
        _corners, ids, _ = _aruco_detector.detectMarkers(image)

        if ids is not NO_ARUCO_FOUND:
            self._aruco_corners = _corners
            int_corners = np.int32(self._aruco_corners) 

            aruco_perimeter = cv2.arcLength(self._aruco_corners[0], True)

            self._pixel_cm_ratio = aruco_perimeter / self.aruco_area

            self._aruco_top_left_x = int_corners[0][0][0][0] ### The point marked by drawDetectedMarkers
            self._aruco_top_left_y = int_corners[0][0][0][1]

            self._aruco_top_right_x = int_corners[0][0][1][0] ### the point in front of the marked point from the drawDetectedMarkers
            self._aruco_top_right_y = int_corners[0][0][1][1]

            self._aruco_bottom_right_x = int_corners[0][0][2][0] 
            self._aruco_bottom_right_y = int_corners[0][0][2][1]

            self._aruco_bottom_left_x = int_corners[0][0][3][0]
            self._aruco_bottom_left_x = int_corners[0][0][3][1]

            self.show_aruco()
        else:
            self._aruco_corners = NO_ARUCO_FOUND
            
        return self._aruco_corners 

    def translate_coordinates(self, coordinates, orientation):
        old_x, old_y = coordinates
        old_angle = orientation

        if self._aruco_corners is not NO_ARUCO_FOUND:
            ### calculat the orientation of the camera
            aruco_vector = np.array([self._aruco_top_right_x - self._aruco_top_left_x, self._aruco_top_right_y - self._aruco_top_left_y]) ### x direction
            image_vector = np.array([1,0])  ### (x, y)

            print(f"top_left: {self._aruco_top_left_x}, {self._aruco_top_left_y}; top_right: {self._aruco_top_right_x}, {self._aruco_top_right_y}")
            tfangle = np.arccos((np.dot(aruco_vector, image_vector)/ (np.linalg.norm(aruco_vector)*np.linalg.norm(image_vector))))   ## cos(theta)=A.B / |A|x|B|
            
            if self._aruco_top_right_y < self._aruco_top_left_y:
                tfangle = -tfangle
            ### tfangle is in radian
            print(f"the angle between aruco upper side and image top line is {np.degrees(tfangle)}")

            tfmatrix1 = np.matrix([[np.cos(tfangle), -np.sin(tfangle), 0 , self._aruco_top_left_x],
                                    [np.sin(tfangle),  np.cos(tfangle), 0 , self._aruco_top_left_y],
                                    [       0       ,         0       , 1 ,        0        ],
                                    [       0       ,         0       , 0 ,        1        ]])
            
            old_coordinate = np.matrix([[old_x],[old_y],[0],[1]])
            new_coordinate = np.matmul(np.linalg.inv(tfmatrix1),old_coordinate)
            new_coordinate = new_coordinate/self._pixel_cm_ratio
            new_angle = old_angle + np.degrees(tfangle)
            if new_angle < 0:
                new_angle = 360+new_angle

            new_angle = round(new_angle)

            cv2.putText(self._image, f'new_angle_inHere: {new_angle}', (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            return new_coordinate, new_angle
        else:
            return NO_ARUCO_FOUND

    def show_aruco(self):
        if self._aruco_corners is not NO_ARUCO_FOUND:
            cv2.aruco.drawDetectedMarkers(self._image, self._aruco_corners)
    def show_aruco_corners(self):
        font_color = {'red':(0, 0, 255), 'green': (0, 255, 0),'orange':(0, 100, 200), 'black':(0,0,0)}

        if self._aruco_corners is not NO_ARUCO_FOUND:
            cv2.circle(self._image, (self._aruco_top_right_x, self._aruco_top_right_y), 2, font_color['orange'], 2)
            cv2.putText(self._image, 'topRight', (self._aruco_top_right_x, self._aruco_top_right_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color['orange'], 2)

            cv2.circle(self._image, (self._aruco_top_left_x, self._aruco_top_left_y), 2, font_color['orange'], 2)
            cv2.putText(self._image, 'topLeft', (self._aruco_top_left_x, self._aruco_top_left_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color['orange'], 2)

            cv2.circle(self._image, (self._aruco_bottom_right_x, self._aruco_bottom_right_y), 2, font_color['orange'], 2)
            cv2.putText(self._image, 'topLeft', (self._aruco_bottom_right_x, self._aruco_bottom_right_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, font_color['orange'], 2)




if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    image_path = current_dir + '/7_ArUco.jpg'
    image = cv2.imread(image_path)
    old_coordinate = (20, 30)
    angle = 20

    aruco_length = 100
    coord_translator = translator(aruco_dictionary=cv2.aruco.DICT_5X5_100, aruco_length=aruco_length)
    coord_translator.aruco_detector(image)
    translation_result = coord_translator.translate_coordinates(old_coordinate, angle)

    if translation_result is NO_ARUCO_FOUND:
        print("NO ARUCO FOUND !!")
    else:
        print(f"new coordinates: {translation_result}")
        cv2.imshow('image', image)
        cv2.waitKey(0)