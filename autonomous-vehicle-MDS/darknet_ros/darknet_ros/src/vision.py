#!/usr/bin/env python
# This code is written at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html

# Usage example:  python3 object_detection_yolo.py --video=run.mp4
#                 python3 object_detection_yolo.py --image=bird.jpg

import cv_bridge
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2, time
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import os.path
import time
import rospy
import rospkg
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32MultiArray


def pub_traffic_sign(mission):
    global list_traffic, list_sign, true, pub_sign, sign_time,traffic_time
    total = 5
    thresh = 4
    if(pub_sign==True):
        if ((mission == 'Red') or (mission == 'Green') or (mission == 'Left') or (mission == 'StraightLeft')):
            traffic_time=time.time()
            list_traffic.append(mission)

        else:
            sign_time=time.time()
            list_sign.append(mission)

        if len(list_traffic) == total:
            R = list_traffic.count('Red')
            G = list_traffic.count('Green')
            L = list_traffic.count('Left')
            SL = list_traffic.count('StraightLeft')
            list_traffic = []
            #print(R,G,L,SL)
            if R >= thresh:
                #send_msg = 'Red'
                print('Red???')
                true.append('Red')
                #print(true)

            elif G >=thresh:
                #send_msg = 'Green'
                print('Green???')
                true.append('Green')

            elif L >=thresh:
                #send_msg = 'Left'
                print('Left???')
                true.append('Left')

            elif SL >=thresh:
                #send_msg = 'StraightLeft'
                print('StraightLeft???')
                true.append('StraightLeft')
        #print(true, list_traffic)
        if len(true)>=2:
            if true[-2]=='Red' and true[-1]=='Red':
                traffic_array.data = [1,0,0,0]
                print('clearly Red')

            elif true[-2]=='Green' and true[-1]=='Green':
                traffic_array.data = [0,1,0,0]
                print('clearly Green')

            elif true[-2]=='Left' and true[-1]=='Left':
                traffic_array.data = [0,0,1,0]
                print('clearly Left')
            elif true[-2]=='StraightLeft' and true[-1]=='StraightLeft':
                traffic_array.data = [0,0,0,1]
                print('clearly StraightLeft')
            #print(send_msg)
            true=[]

            pub_traffic.publish(traffic_array)
    else:
        if (time.time()-sign_time>=3):
            #true=[]
            list_sign=[]
        if (time.time()-traffic_time>=3):
            true=[]
            list_traffic=[]

        #print(true, list_traffic, list_sign)
def BoundingBoxes_callback(data):
    global label, pub_sign

    label=data.bounding_boxes[0].Class
    pub_sign=True

if __name__ == '__main__':
    list_traffic = []
    list_sign = []
    true = []
    sign_time=0
    traffic_time=0
    pub_sign=True
    label=""

    rospy.init_node('Traffic', anonymous=True)

    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, BoundingBoxes_callback)
    pub_traffic = rospy.Publisher('/detect/traffic_sign', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    traffic_array=Int32MultiArray()
    traffic_array.data=[0,0,0,0]
    while (True):
        try:
            pub_traffic_sign(label)
            pub_sign=False
            if cv2.waitKey(1) == ord('q'):
                break

        except rospy.ROSInterruptException:
            pass
