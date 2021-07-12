#! /usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
import math

class StopLine():
    def __init__(self):
        rospy.init_node('Stop_node', anonymous=True)
        self.rate = 0
        self.image = None
        self.pub_control = rospy.Publisher('stop_line', Int32, queue_size=5)
        self.stop_sliced_img = rospy.Publisher('sliced_image', Image, queue_size=5)
        self.stop_result_img = rospy.Publisher('stop_result_image', Image, queue_size=5)
        self.sub_image = rospy.Subscriber('/video/lane', Image, self.img_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        #cap.set(cv2.CAP_MSMF,4000)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.Brightness = 0
        self.Focus = 0
        self.TH_value = 180
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, False)
        self.cap.set(28, self.Focus)
        self.thresh_img = None

    def img_callback(self, data):
        try:
            # Read the image from the input topic:
            self.image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e


    def convert_gray_scale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


    def slice_image(self, image, height):
        # src = cv2.imread(image,cv2.IMREAD_COLOR)
        #cv2.imshow("image", image)
        # dst = src.copy()
        resize_image=cv2.resize(image,(1280,720))
        slicedimage = resize_image[600:(600 + height), 350:1000]


        return slicedimage

    def select_white(self, image, image_hist):
        gray = cv2.cvtColor(image_hist, cv2.COLOR_RGB2GRAY)
        gray2 = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


        hist = cv2.calcHist([gray],[0],None,[256],[0,256])

        sum_hist = 0
        for i in range(0,len(hist)):
            sum_hist = sum_hist + hist[i]*i
        mean = int(round(sum_hist/sum(hist)))

        # lower_white = np.array([mean, mean, mean])
        # upper_white = np.array([255, 255, 255])
        mask_gray = cv2.inRange(gray2, mean+20, 255)

        # plt.ion()
        # plt.cla()
        # plt.plot(hist,color='r')
        # plt.xlim([0,256])
        # plt.show()
        # plt.pause(0.01)
        
        return mask_gray


    def adaptive_thresholding(self, image):
        ret, img = cv2.threshold(image, self.TH_value, 255, cv2.THRESH_BINARY)
        # ret, img = cv2.threshold(image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # value = (1,220),(2,180),(3,140),(4,180),(5,180)
        return img


    def get_pixel_value(self, thresholding):
        #sumx=thresholding.sum(1)
        #print (sum)

        dot_np = np.copy(thresholding)
        dotindex = np.where(dot_np == 255)
        #dotindex_x = dotindex[1]
        #length = len(dotindex_x)

 
        l1 = float(len(dotindex[1]))
        l2 = float(len(dot_np)*len(dot_np[0]))
        self.rate = l1/l2


        if self.rate>0.95:
            print (round(self.rate,2), '1')
            return 1
        else:
            print (round(self.rate,2), '0')
            return 0



    def Camera(self, frame):
        slicedimage_hist = self.slice_image(frame.copy(), 80)
        slicedimage = self.slice_image(frame.copy(), 30)
        self.stop_sliced_img.publish(self.bridge.cv2_to_imgmsg(slicedimage))

        white_image = self.select_white(slicedimage, slicedimage_hist)
        # cv2.imshow("white_image", white_image)
        # masked_image = select_region(white_yellow_image)

        #gray_scale = self.convert_gray_scale(white_image)

        # self.thresh_img = self.adaptive_thresholding(white_image)
        self.thresh_img = white_image


        sign = self.get_pixel_value(self.thresh_img)

        return sign



if __name__ == '__main__':
    # Init the consumer:
    stopline = StopLine()
    # Continue to spin
    while not rospy.is_shutdown():
        try:
            if(stopline.image is not None):
                #ret, frame = stopline.cap.read()
                #frame = cv2.imread('/home/rnd/Pictures/Screenshot from 2020-08-27 17-23-11.png', cv2.IMREAD_COLOR)
                frame = stopline.image

                Flag = stopline.Camera(frame)
                stopline.stop_result_img.publish(stopline.bridge.cv2_to_imgmsg(stopline.thresh_img))
                #if(stopline.rate > 0.72):
                #    rospy.sleep(1)

                stopline.pub_control.publish(Flag)
                rospy.sleep(0.05)

            
       
        except KeyboardInterrupt:
            print "Shutting down vison node."