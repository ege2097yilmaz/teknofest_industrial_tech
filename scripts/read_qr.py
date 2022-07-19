#!/usr/bin/env python3

import cv2, rospy, logging, cv_bridge, time
from pyzbar import pyzbar

from sensor_msgs.msg import Image
from teknofest_industrial_tech.msg import qr
from nav_msgs.msg import Odometry

class qrReader:
    def __init__(self):
        # defining cv_bridge to convert ros topic to cv numpy array
        self.bridge = cv_bridge.CvBridge()

        # defining qr object
        self.qr1 = qr()

        # defining initial velocties
        self.x_vel = 0
        self.y_vel = 0
        self.z_vel = 0

        # setting parameters
        self.wait_for_qr = rospy.get_param("/wait_for_qr", 1.5)
        self.qr_image_topic = rospy.get_param("/qr_image_topic", "/usb_cam2/image_raw")
        self.show_qr_image = rospy.get_param("/show_qr_image", False)
        self.odom_topic = rospy.get_param("/odom", "/odom")

        self.counter = 0
        self.qr_index = 0

        # definning publisher and subscribers
        self.image_sub = rospy.Subscriber(self.qr_image_topic, Image, self.video)
        self.qr_pub = rospy.Publisher('/qr_state', qr, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom)

        # run video_edit func
        self.video_edit()

    # for filtering of qr reading, use the odometry datas
    def odom(self, odom):
        self.x_vel = odom.twist.twist.linear.x
        self.y_vel = odom.twist.twist.linear.y
        self.z_vel = odom.twist.twist.angular.z

    def read_barcodes(self, frame):
        barcodes = pyzbar.decode(frame)

        # marking the barcode on the screen
        for barcode in barcodes:
            x, y, w, h = barcode.rect

            barcode_info = barcode.data.decode("utf-8")
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, barcode_info, (x + 6, y - 6), font, 2.0, (255,255,255), 1)
            
            
        if barcodes:
            return [True, frame]
        
        else:
            return [False, frame]
            
    # callback for image came from camera 
    def video(self, images):
        # converting topic to cv2 arrays
        self.goruntu = self.bridge.imgmsg_to_cv2(images, 'bgr8') 

        # show qr code frame
        if self.show_qr_image == True:
            cv2.imshow("qr", self.goruntu)
            cv2.waitKey(3)
    
    def video_edit(self):
        rospy.sleep(5)
        while not rospy.is_shutdown():
            try:
                ##########READ QR CODE FUNCTION RETURN BOOL##########
                if self.x_vel > 0.0 and self.z_vel < 0.15 :
                    qr_bool = self.read_barcodes(self.goruntu)[0]
                    if qr_bool == True:
                        rospy.sleep(self.wait_for_qr)
                        self.qr_index +=1
                    # qr_meaning = self.read_barcodes(goruntu)[1]
                else:
                    qr_bool = False
            except ValueError:
                print(ValueError)

            # when camera see a qr then send as a TRUE
            if qr_bool == True:
                rospy.logdebug("qr detected and waiting for next qr")
                self.counter += 1
                if self.counter > 1: 
                    rospy.logerr(self.qr_index)
                    if self.qr_index ==2:
                        self.qr1.stamp = rospy.Time.now()
                        self.qr1.qr = True
                        self.qr1.qr_name = "rq detected from camera"
                        self.qr_pub.publish(self.qr1)
                        self.counter = 0
                        self.qr_index = 0
                
            else:
                self.qr1.stamp = rospy.Time.now()
                self.qr1.qr = False
                self.qr1.qr_name = "qr is not detetced from camera"
                self.qr_pub.publish(self.qr1)

if __name__ == "__main__":
    rospy.init_node('qr_reader_node', anonymous=True)
    rospy.loginfo("qr_reader_node is starting ...")

    #activitng functions for ros debugger
    logger = logging.getLogger("rosout") #setting output broker of debug 
    logger.setLevel(logging.DEBUG)
    level = logger.getEffectiveLevel()

    rate = rospy.Rate(5) #hz
    qr_object = qrReader()
    rospy.spin()