#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import rospy, cv2, logging
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image
from teknofest_industrial_tech.msg import linePoint

class lane:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_topic = rospy.get_param("/image_topic", "/usb_cam/image_raw")
        self.show_image = rospy.get_param("/show_image", False)
        
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.fonksiyon) #it is for gazenbo
        # self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.fonksiyon) # it is for usb_cam
        self.pointPub = rospy.Publisher('/points', linePoint, queue_size=10)

    def fonksiyon(self, ros_goruntu):

        # converting topic to cv2 arrays
        cv_goruntu = self.bridge.imgmsg_to_cv2(ros_goruntu, 'bgr8') 

        hsv_goruntu = cv2.cvtColor(cv_goruntu, cv2.COLOR_BGR2HSV)

        # detect yellow color
        # lower_yellow = np.array([10, 10, 10])
        # upper_yellow = np.array([255, 255, 250])

        # detect black color
        lower_yellow = np.array([126, 67, 23], np.uint8)
        upper_yellow = np.array([146, 87, 103], np.uint8)

        mask = cv2.inRange(hsv_goruntu, lower_yellow, upper_yellow)

        h, w, d = hsv_goruntu.shape

        top = 3*h/4
        botom = 3*h/4 + 300

        left = w/4 
        right = 3*w/4 
        w_half = w/2

        mask[0:int(top), 0:w] = 0
        mask[int(botom):h, 0:w] = 0
        
        mask[0:int(left), 0:h] = 0
        mask[int(right):h, 0:h] = 0

        # cv_goruntu[0:int(top), 0:w] = 0
        # cv_goruntu[int(botom):h, 0:w] = 0

        # cv_goruntu[0:h, 0:int(left)] = 0
        # cv_goruntu[0:h, int(right):w] = 0

  
        moment = cv2.moments(mask)
        if moment['m00'] > 0:
            cx = int(moment['m10']/moment['m00'])
            cy = int(moment['m01']/moment['m00'])

            cv2.circle(cv_goruntu, (cx, cy), 20, (0,255,100), -1)

            if self.show_image == True:
                cv2.imshow('merkez', cv_goruntu)
                cv2.waitKey(3)

            lpoints = linePoint()
            lpoints.cx = cx
            lpoints.cy = cy
            lpoints.w = w_half
            lpoints.avaiable = True
            self.pointPub.publish(lpoints)
            # rospy.loginfo("[INFO] lane detected")

        else:
            lpoints = linePoint()
            lpoints.avaiable = False
            self.pointPub.publish(lpoints)
            # rospy.logwarn("[WARN] lane cannot be detected")

if __name__ == "__main__":
    rospy.init_node('lane_detection', anonymous=True)

    #activitng functions for ros debugger
    logger = logging.getLogger("rosout") #setting output broker of debug 
    logger.setLevel(logging.DEBUG)
    level = logger.getEffectiveLevel()

    obje = lane()
    rospy.spin()