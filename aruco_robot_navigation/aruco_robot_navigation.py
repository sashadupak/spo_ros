import numpy as np
import cv2
from cv2 import aruco
import json
from math import *
from statistics import mean

camera_parameters_filename = "../camera_calibration/camera_param1.json"

# markers coordinates
l = 1
world_markers = {0: (0, 0), 
                 1: (l, 0),
                 2: (0, l),
                 3: (l, l)}
world_marker_size = 75/1000

robot_markers = {4: (0, 0.04),
                 5: (0, -0.05)}
robot_marker_size = 75/1000

# robot dimentions
a = 0.2
b = 0.25
c = 0.12

with open(camera_parameters_filename, "r") as read_file:
    decodedArray = json.load(read_file)
    mtx = np.asarray(decodedArray["mtx"])
    dist = np.asarray(decodedArray["dist"])

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

class ArucoRobotNavigation:

    def inversePerspective(self, rvec, tvec):
        """ Applies perspective transform for given rvec and tvec. """
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(R, np.matrix(-tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        """ Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        invRvec, invTvec = self.inversePerspective(rvec2, tvec2)

        info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
        composedRvec, composedTvec = info[0], info[1]

        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))
        return composedRvec, composedTvec        

    def get_robot_position(self, img):
        if img is None:
            print("no image")
            return None, img

        h, w = img.shape[:2]
        newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        img = cv2.undistort(img, mtx, dist, None, newCameraMtx)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        img = aruco.drawDetectedMarkers(img.copy(), corners, ids)

        world_markers_detected = 0
        robot_markers_detected = 0
        sum_worldTvec = np.zeros((3, 1))
        sum_worldRvec = np.zeros((3, 1))
        sum_botTvec = np.zeros((3, 1))
        sum_botRvec = np.zeros((3, 1))
        if ids is not None:
            for i in range(len(ids)):
                c = corners[i][0]
                if int(ids[i]) in world_markers.keys():
                    world_markers_detected += 1
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers([c], world_marker_size, mtx, dist) 
                    aruco.drawAxis(img, mtx, dist, rvec, tvec, world_marker_size)

                    rvec, tvec = rvec.reshape((3, 1)), tvec.reshape((3, 1))
                    tvec[0] += world_markers[int(ids[i])][0]
                    tvec[1] += world_markers[int(ids[i])][1]
                    sum_worldTvec += tvec
                    sum_worldRvec += rvec

                elif int(ids[i]) in robot_markers.keys():
                    robot_markers_detected += 1
                    botRvec, botTvec, _ = aruco.estimatePoseSingleMarkers([c], robot_marker_size, mtx, dist)
                    aruco.drawAxis(img, mtx, dist, botRvec, botTvec, robot_marker_size)

                    botRvec, botTvec = botRvec.reshape((3, 1)), botTvec.reshape((3, 1))
                    botTvec[0] += robot_markers[int(ids[i])][0]
                    botTvec[1] += robot_markers[int(ids[i])][1]
                    sum_botTvec += botTvec
                    sum_botRvec += botRvec

        if (world_markers_detected > 0) and (robot_markers_detected > 0):
            avg_worldTvec = sum_worldTvec/world_markers_detected
            avg_worldRvec = sum_worldRvec/world_markers_detected
            avg_botTvec = sum_botTvec/robot_markers_detected
            avg_botRvec = sum_botRvec/robot_markers_detected

            composedRvec, composedTvec = self.relativePosition(avg_botRvec, avg_botTvec, avg_worldRvec, avg_worldTvec)

            x = float(composedTvec[0])
            y = float(composedTvec[1])
            phi = float(composedRvec[2])

            return [x, y, phi], img
        else:
            #print("not enough markers detected")
            return None, img 


if __name__ == "__main__":
    aruco_nav = ArucoRobotNavigation()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("no video")
        exit()
    
    while True:
        ret, img = cap.read()
        if img is None:
            break

        aruco_pos, img = aruco_nav.get_robot_position(img)
        if aruco_pos is not None:
            print(aruco_pos)

        cv2.imshow('img', img)
        key = cv2.waitKey(33)
        if (key == ord("q")) or (key == 27):
            break
    cap.release()
    cv2.destroyAllWindows()
