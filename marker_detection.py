#! /usr/bin/env python2

import rospy
import mavros
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Float32, String
from tf import transformations
from object_detection import ObjectDetection
import mavros_msgs.msg

class MarkerDetector:
    def __init__(self):
        ### Init node ###
        rospy.init_node('marker_detector')
        mavros.set_namespace('mavros')

        ### YOLO image ###
        self.OD = ObjectDetection()
        video_sub = rospy.Subscriber("darknet_ros/detection_image" , Image, callback = self.OD.image_callback)

        ### Camera image ###
        image_subscriber = rospy.Subscriber("usb_cam/image_raw", Image, self.image_callback)

        ### Object detection and marker detection image ###
        self.ODMD_detection_img_pub = rospy.Publisher('/ODMD_detection_img', Image, queue_size = 10)
        
        self.bridge = CvBridge()
        
        ### Set up publishers ###
        self.aruco_marker_img_pub = rospy.Publisher('/aruco_marker_img', Image, queue_size = 10)

        ### Aruco_data connect ###
        self.aruco_x_pub = rospy.Publisher("aruco_x", String, queue_size = 10)
        self.aruco_y_pub = rospy.Publisher("aruco_y", String, queue_size = 10)
        self.aruco_area_pub = rospy.Publisher("aruco_area", String, queue_size = 10)
        self.aruco_id_pub = rospy.Publisher("aruco_id", String, queue_size = 10)

        ### Initialize variables ###
        self.frame = np.zeros((240, 320, 3), np.uint8)
        self.pos = [0.0] * 4
        self.markerPos = [0.0] * 4
        self.corners = np.zeros((4, 2))
        self.aruco_x = 0
        self.aruco_y = 0
        self.aruco_area = 0
        self.id = -1

        ### camera intristic parameters ###
        self.K = np.array([277.191356, 0.0, 160.5, 0.0, 277.191356, 120.5, 0.0, 0.0, 1.0]).reshape(3,3)
        self.distCoeffs = np.array([0.0] * 5)

        ### Load the dictionary that was used to generate the markers ###
        self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

        ### Initialize the detector parameters using default values ###
        self.parameters =  cv.aruco.DetectorParameters_create()
        
        ### Setup rate ###
        self.rate = rospy.Rate(20)
        self.rate.sleep()

    def image_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def Aruco_marker_detector(self):
        ### YOLO frame ###
        ODMD_img = self.OD.frame

        ### Aruco frame ###
        img = self.frame.copy() 
        self.aruco_marker_img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        
        ### Detect the markers in the image ###
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)

        if(markerCorners):
            self.frame = img
            self.id = markerIds[0]
            try:
                self.corners = np.array(markerCorners).reshape(4, 2)
            except:
                rospy.loginfo("markerCorners")
        else:
            self.id = -1
            #rospy.loginfo("Not detect marker")

        ### Draw the Bounding box and conter point ###
        if(self.id == 23):
            try:
                (topLeft, topRight, bottomRight, bottomLeft) = self.corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomRight[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv.line(ODMD_img, topLeft, topRight, (0, 255, 0), 2)
                cv.line(ODMD_img, topRight, bottomRight, (0, 255, 0), 2)
                cv.line(ODMD_img, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv.line(ODMD_img, bottomLeft, topLeft, (0, 255, 0), 2)

                cx = int((topLeft[0] + bottomRight[0]) / 2.0)
                cy = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv.circle(ODMD_img, (cx, cy), 4, (0, 0, 255), -1)
                cv.putText(ODMD_img, str(MD.id), (topLeft[0], topLeft[1] -15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                marker_length1 = int((((int(topLeft[0])-int(topRight[0]))**2) + ((int(topLeft[1])-int(topRight[1])))**2)**0.5)
                marker_length2 = int((((int(topRight[0])-int(bottomRight[0]))**2) + ((int(topRight[1])-int(bottomRight[1])))**2)**0.5)
                self.aruco_x = cx
                self.aruco_y = cy
                self.aruco_area = marker_length1 * marker_length2
                self.aruco_id = 23
            except:
                print("Draw error")
        else:
            self.aruco_x = 0
            self.aruco_y = 0
            self.aruco_area = 0
            self.aruco_id = -1
        
        ### Publish ###
        self.ODMD_detection_img_pub.publish(self.OD.bridge.cv2_to_imgmsg(ODMD_img, "bgr8"))
        self.aruco_x_pub.publish(str(self.aruco_x))
        self.aruco_y_pub.publish(str(self.aruco_y))
        self.aruco_area_pub.publish(str(self.aruco_area))
        self.aruco_id_pub.publish(str(self.aruco_id))

        self.rate.sleep()

if __name__ == "__main__":
    MD = MarkerDetector()  
    rospy.loginfo("Marker detect...")
    while not rospy.is_shutdown():
        MD.Aruco_marker_detector()