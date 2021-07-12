#! /usr/bin/env python3
#import keyboard
import serial
import socket as soc
import rospy
import time
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from ntrip.NtripClient import *

import multiprocessing
from multiprocessing import Process, Queue

import os
import sys

import math
port = str(rospy.get_param("~gps_port","/dev/ttyACM0"))
gps_data_bef = ""

class SocketInfo():
    HOST=""
    PORT=7777
    BUFSIZE=10

    def __init__(ADDRESS = "127.0.0.1"):
        HOST = ADDRESS

    ADDR=(HOST, PORT)

def cb_imu(data):
    yaw = data



def do_work(lat,lon):
    gpsmsg.header.stamp = rospy.Time.now()
    gpsmsg.header.frame_id = "GPS_link"
    gpsmsg.latitude=lat
    gpsmsg.longitude=lon
    gpsmsg. position_covariance_type=0
    gps_pub.publish(gpsmsg)
    #print(gpsmsg)

def pub_accuracy(accuracy):
    global prev_accuracy
    current_gps_accuracy=0

    if (accuracy==1):
        current_gps_accuracy=0
    elif (accuracy==2):
        current_gps_accuracy=1
    elif (accuracy==5):
        current_gps_accuracy=2
    elif (accuracy==4):
        current_gps_accuracy=3

    gps_accuracy_pub.publish(current_gps_accuracy)

fix_type={ '0' : "Invalid",
           '1' : "GPS fix (SPS)",
           '2' : "DGPS fix",
           '3' : "PPS fix",
           '4' : "Real Time Kinematic",
           '5' : "Float RTK",
           '6' : "estimated (dead reckoning) (2.3 feature)",
           '7' : "Manual input mode",
           '8' : "Simulation mode"}


if __name__ == '__main__':
    rospy.init_node("gps_node")

    port = rospy.get_param("~GPS_PORT",port)
    #print(port)

    TCP_info = SocketInfo()
    TCP_sock = soc.socket(soc.AF_INET, soc.SOCK_STREAM)
    #print("main : {}".format(os.getpid()))

    ntripArgs = {}
    ntripArgs['lat']=37.236134
    ntripArgs['lon']=126.774126
    #SUWON
    #ntripArgs['lat']=37.630873
    #ntripArgs['lon']=127.076533
    #SOUL

    #ntripArgs['lat']=37.16
    #ntripArgs['lon']=127.30
    #SUWON
    #ntripArgs['lat']=37.630873
    #ntripArgs['lon']=127.076533
    #SOUL

    ntripArgs['height']=0
    ntripArgs['host']=False
    ntripArgs['ssl']=False

    #ntripArgs['user']="gnss"+":"+"gnss"
    ntripArgs['user']="agc770@naver.com"+":"+"gnss"
    #ntripArgs['user']="agc77000"+":"+"ngii"
    ntripArgs['caster']="gnssdata.or.kr"
    #ntripArgs['caster']="vrs3.ngii.go.kr"
    ntripArgs['port']=int("2101")
    #ntripArgs['port']=int("2201")

    ntripArgs['mountpoint']="SUWN-RTCM32"
    #ntripArgs['mountpoint']="SOUL-RTCM32"
    #ntripArgs['mountpoint']="VRS-RTCM31"
    #ntripArgs['mountpoint']="FKP-RTCM31"

    ntripArgs['V2']=True

    ntripArgs['verbose']=False
    ntripArgs['headerOutput']=None

    maxReconnect=1
    maxConnectTime=1200
    ser = serial.serial_for_url(port,115200, timeout=0)
    multiprocessing.set_start_method('spawn', True)
    nclient = NtripClient(**ntripArgs)
    que = Queue()
    que_pos = Queue()

    proc = Process(target=nclient.update_RTK, args=(que,que_pos,))

    proc.start()
    isrunning=True
    isReady=False
    count_ready = 0
    t=time.time()
    prev_pos = [0.0,0.0]
    Line = 0.0
    prev_time=0
    reRTK_count=True
    prev_accuracy=0

    gps_degree=0
    gps_speed=0

    gps_pub=rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    gps_accuracy_pub=rospy.Publisher('/gps/accuracy', UInt32, queue_size=10)
    gpsmsg=NavSatFix()

    gps_cog_pub=rospy.Publisher('/gps/cog', Float32, queue_size=10)

    rospy.loginfo("initialised")

    r=rospy.Rate(1)

    #yaw_sub = rospy.Subscriber("yaw_imu",Quaternion,cb_imu)


    while isrunning:
        RoverMessage=ser.readline().decode('ascii')

        if que.empty()==False:
            data = que.get()[0]
            #print(type(data))

            if (type(data) is bool):
                pass
            else:
                ser.write(data)
                #print(1)


        '''
        if que.empty()==False:
            data = que.get()[0]
            ser.write(data)
        '''
        t = time.time()
        #print(RoverMessage)
        try:
            #print(RoverMessage)
            if "RMC" in RoverMessage:
                data=RoverMessage.split(",")

                gps_speed = round(float(data[7]),8)
                gps_degree = round(float(data[8]),8)
                #gps_degree = -float(data[8])+90
                #if(gps_degree<-180):
                #    gps_degree+=360
                print(gps_speed,gps_degree)
                gps_cog_pub.publish(gps_degree)


            if "GGA" in RoverMessage:
                data=RoverMessage.split(",")

                lat = round(float(data[2])*0.514444,8) #knot to m/s
                lon = round(float(data[4]),8)
                #print(data)
                #print(lat, lon)
                lat_str = str(data[2]); lon_str = str(data[4])


                deg_lat = int(float(lat_str)/100)
                deg_lon = int(float(lon_str)/100)

                minute_lat = float(lat_str)%100
                minute_lon = float(lon_str)%100

                #print(deg_lat, deg_lon, minute_lat, minute_lon)

                lat_degree = round(deg_lat + minute_lat/60,8)
                lon_degree = round(deg_lon + minute_lon/60,8)

                #print("{} {}".format(lat_degree,lon_degree))

                print ("Fix Type : %s  North : %.7f  East : %.7f \r"% (fix_type[data[6]],lat_degree,lon_degree))
                #print(data[6])
                que_pos.put([lat,lon])

                nclient.setPosition(lat_degree,lon_degree)
                #if(isReady==False): print(Line)
                #print(data[6])
                if (int(data[6]) >=1):
                    do_work(lat_degree,lon_degree)
                    pub_accuracy(int(data[6]))


                if (int(data[6])==1):
                    #print(proc)
                    print(reRTK_count)
                    if reRTK_count:
                        print("retry RTK Mode !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        reRTK_count=False
                        prev_time=time.time()

                        que = Queue()
                        que_pos = Queue()

                        proc = Process(target=nclient.update_RTK, args=(que,que_pos,))
                        proc.start()

                if time.time()-prev_time>=5:
                    reRTK_count=True
                    



        except:
            print ("Missed" ,"\r")
        #mainloop()
