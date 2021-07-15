#! /usr/bin/env python

import rospy
import numpy as np
from cv_bridge import CvBridge
import sys, select, termios, tty, math
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2, time
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from sensor_msgs.msg import Image

def GetKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def filter_region(image, vertices):
    mask = np.zeros_like(image)
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])
    return cv2.bitwise_and(image, mask)

def select_region(image):
    bl_x=0.0
    bl_y=1.0
    br_x=0.85
    br_y=1.0
    tl_x=0.0
    tl_y=0.0
    tr_x=0.85
    tr_y=0.0
    rows, cols = image.shape[:2]
    bottom_left = [cols*bl_x, rows*bl_y]
    top_left    = [cols*tl_x, rows*tl_y]
    bottom_right= [cols*br_x, rows*br_y]
    top_right   = [cols*tr_x, rows*tr_y]


    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)

    a=filter_region(image, vertices)

    return a

if __name__ == '__main__':
    rospy.init_node('camera_node')

    capture1 = cv2.VideoCapture(0)
    capture2 = cv2.VideoCapture(1)
    #capture3 = cv2.VideoCapture(2)
    capture1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #capture3.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
    #capture3.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
    image_a='/video/lane'
    image_b='/video/sign'
    video_pub1 = rospy.Publisher(image_a, Image, queue_size=5)
    video_pub2 = rospy.Publisher(image_b, Image, queue_size=5)
    #video_pub3 = rospy.Publisher('/video/parking', Image, queue_size=5)

    bridge = CvBridge()
    settings = termios.tcgetattr(sys.stdin)
    time.sleep(1)
    prev_time=0
    
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
    #while (capture1.isOpened() and capture2.isOpened() and capture3.isOpened()):
    #compress
        try:
            if(capture1.isOpened()):
                ret, frame1 = capture1.read()
                #cv2.imshow(image_a, frame1)
                
                #cv2.imshow("image_after", frame1)
                a=bridge.cv2_to_imgmsg(frame1, "bgr8")

                video_pub1.publish(a)
            
            if(capture2.isOpened()):
                ret2, frame2 = capture2.read()
                #cv2.imshow(image_b, frame2)
                frame2 = select_region(frame2)
                b=bridge.cv2_to_imgmsg(frame2, "bgr8")

                video_pub2.publish(b)
            '''
            if(capture3.isOpened()):
                ret3, frame3 = capture3.read()
                cv2.imshow("VideoFrame3", frame3)
            
                c=bridge.cv2_to_imgmsg(frame3, "bgr8")

                video_pub3.publish(c)
            '''

            #aa=bridge.imgmsg_to_cv2(a)
            #cv2.imshow("aa",aa)
            #print("FPS: ",1/(time.time()-prev_time))
            prev_time=time.time()

            #key=GetKey()
            #if key == '\x03': #CTRL + C
            #    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            r.sleep()

        except rospy.ROSInterruptException:
            pass
        
    capture1.release()
    capture2.release()
    #capture3.release()
    cv2.destroyAllWindows() 
