#! /usr/bin/env python3
#import keyboard
import serial
import socket as soc
import rospkg
#import rospy
#import keyboard
import time
from ntrip.NtripClient import *

import multiprocessing
from multiprocessing import Process, Queue

import os
import sys

import math

#yaw = Quaternion()
#pos = PointStamped()
gps_data_bef = ""
prev_time=0



def convert_degree_to_meter(lat_data,lon_data):
    alpha = lat_data*math.pi/180
    beta = lon_data*math.pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *math.pi/180
    beta0 = (125+0.002890277)*math.pi/180

    T = pow(math.tan(alpha),2)
    C = e1/(1-e1)*pow(math.cos(alpha),2)
    AA = (beta-beta0)*math.cos(alpha)  #both radian

    N = a/math.sqrt( 1-e1*math.sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-math.sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +math.sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*math.sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-math.sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +math.sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*math.sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*math.tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]


class SocketInfo():
    HOST=""
    PORT=7777
    BUFSIZE=10

    def __init__(ADDRESS = "127.0.0.1"):
        HOST = ADDRESS

    ADDR=(HOST, PORT)

def cb_imu(data):
    yaw = data

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

    #ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
    #print(port)

    TCP_info = SocketInfo()
    TCP_sock = soc.socket(soc.AF_INET, soc.SOCK_STREAM)
    #print("main : {}".format(os.getpid()))

    ntripArgs = {}
    #ntripArgs['lat']=37.236134
    #ntripArgs['lon']=126.774126
    #SUWON
    #ntripArgs['lat']=37.630873
    #ntripArgs['lon']=127.076533
    #SOUL
    #ntripArgs['lat']=37.16
    #ntripArgs['lon']=127.30
    #SUWON
    ntripArgs['lat']=37.630873
    ntripArgs['lon']=127.076533
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

    #ntripArgs['mountpoint']="SUWN-RTCM31"
    ntripArgs['mountpoint']="SOUL-RTCM31"
    #ntripArgs['mountpoint']="VRS-RTCM31"
    #ntripArgs['mountpoint']="FKP-RTCM31"

    ntripArgs['V2']=True

    ntripArgs['verbose']=False
    ntripArgs['headerOutput']=None

    maxReconnect=1
    maxConnectTime=1200
    ser = serial.serial_for_url('/dev/ttyACM0',115200, timeout=0)
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
    prev_time_rtk=0
    reRTK_count=True
    prev_time=0
    prev_time_rtk=0
    prev_pos = [0.0,0.0]
    Line = 0.0

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"
    f=open(arg_name+"gps_data_seoultech.txt",'w')

    while isrunning:
        RoverMessege=ser.readline().decode('ascii')

        if que.empty()==False:
            data = que.get()[0]

            if (type(data) is bool):
                pass
            else:
                ser.write(data)


        '''
        if que.empty()==False:
            data = que.get()[0]
            ser.write(data)
        '''

        t = time.time()
        #print(RoverMessege)

        try:
            if "GGA" in RoverMessege:
                data=RoverMessege.split(",")

                lat = round(float(data[2]),8)
                lon = round(float(data[4]),8)
                #print(lat, lon)
                lat_str = str(data[2]); lon_str = str(data[4])


                deg_lat = int(float(lat_str)/100)
                deg_lon = int(float(lon_str)/100)

                minute_lat = float(lat_str)%100
                minute_lon = float(lon_str)%100

                #print(deg_lat, deg_lon, minute_lat, minute_lon)

                lat_degree = round(deg_lat + minute_lat/60,7)
                lon_degree = round(deg_lon + minute_lon/60,7)

                #print("{} {}".format(lat_degree,lon_degree))
                print ("Fix Type : %s  North : %.7f  East : %.7f \r"% (fix_type[data[6]],lat_degree,lon_degree))

                que_pos.put([lat,lon])
                pos_data = convert_degree_to_meter(lat_degree,lon_degree)

                Line = math.sqrt((pos_data[1]-prev_pos[1])**2 + (pos_data[0]-prev_pos[0])**2)
                prev_pos = pos_data

                if(Line<=0.2) : count_ready=count_ready+1
                else: count_ready = 0

                if(count_ready>=10) : isReady=True

                nclient.setPosition(lat,lon)
                #if(isReady==False): print(Line)
                if ((float(data[6])==4) or (float(data[6])==5) or (float(data[6])==2) or (float(data[6])==1)):
                    if (time.time()-prev_time>=1):
                        print(lat_degree,lon_degree)
                        f.write(str(lat_degree))
                        f.write(','+str(lon_degree)+'\n')
                        prev_time=time.time()
                        #print('good!')

                if (int(data[6])==1):
                    #print(proc)
                    print(reRTK_count)
                    if reRTK_count:
                        print("retry RTK Mode !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        reRTK_count=False
                        prev_time_rtk=time.time()

                        que = Queue()
                        que_pos = Queue()

                        proc = Process(target=nclient.update_RTK, args=(que,que_pos,))
                        proc.start()

                if time.time()-prev_time_rtk>=5:
                    reRTK_count=True

                if keyboard.is_pressed('esc'):
                    print('save ok')
                    f.close()
                    break

        except:
            pass
            #print ("Missed" ,"\r")
        #mainloop()
