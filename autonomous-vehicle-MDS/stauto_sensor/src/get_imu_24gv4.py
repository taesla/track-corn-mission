#! /usr/bin/env python

import serial
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

port = str(rospy.get_param("~imu_port","/dev/ttyUSB0"))
rpy=[0,0,0]
w_speed=[0,0,0]
accel=[0,0,0]
error_yaw=0 #non magnetic -201.5
#error_yaw=-92.6
def euler_to_quaternion(roll,pitch,yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def imu_error_callback(data):
    global error_yaw, rpy
    error_yaw +=-rpy[2] + data.data
    #print(error_yaw)


def pub_tf_transform(roll,pitch,yaw,w_speed,accel):


    t = TransformStamped()  # pose of turntable_frame w.r.t. turntable_base
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'base_link'
    t.child_frame_id  = 'imu_link'
    t.transform.translation.x = roll
    t.transform.translation.y = pitch
    t.transform.translation.z = yaw
    q = euler_to_quaternion(roll, pitch, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node("get_imu")

    port = rospy.get_param("~GPS_PORT",port)
    ser = serial.serial_for_url(port,115200, timeout=0)

    rospy.Subscriber("yaw_error",Float32,imu_error_callback)
    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

    br = tf2_ros.TransformBroadcaster()

    #r=rospy.Rate(20)

    imu=Imu()

    while not rospy.is_shutdown():
        IMU_message=ser.readline()

        if (len(IMU_message)>55):
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "imu_link"
            data=IMU_message.split(",")
            #print(data)
            rpy[0]=round(float(data[1]),3)
            rpy[1]=round(float(data[2]),3)
            rpy[2]=round(float(data[3]),3)+error_yaw
            #print(rpy[2])
            if (rpy[2] >= 180):
                rpy[2] = rpy[2] - 2*180
            elif (rpy[2] <= -180):
                rpy[2] = rpy[2] + 2*180

            roll=rpy[0]*np.pi/180
            pitch=-rpy[1]*np.pi/180
            yaw=-rpy[2]*np.pi/180

            qx,qy,qz,qw = euler_to_quaternion(roll, pitch, yaw)

            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw

            w_speed[0]=round(float(data[4]),3)
            w_speed[1]=round(float(data[5]),3)
            w_speed[2]=round(float(data[6]),3)

            imu.angular_velocity.x = w_speed[0]
            imu.angular_velocity.y = w_speed[1]
            imu.angular_velocity.z = w_speed[2]

            accel[0]=round(float(data[7]),3)
            accel[1]=round(float(data[8]),3)
            accel[2]=round(float(data[9]),3)

            imu.linear_acceleration.x = accel[0]
            imu.linear_acceleration.y = accel[1]
            imu.linear_acceleration.z = accel[2]

            battery=round(float(data[10]),3)
            imu_pub.publish(imu)
            pub_tf_transform(roll,pitch,yaw,w_speed,accel)
            print("yaw:",rpy[2], "battery:",battery )
            #r.sleep()

        else:
            pass
    else:
        pass
        #mainloop()
