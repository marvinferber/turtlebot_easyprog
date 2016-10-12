#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Marvin Ferber
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import threading
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from types import *


publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1) 

def drehen(drehgeschwindigkeit=1.0, dauer=1.0):
    assert type(drehgeschwindigkeit) is FloatType, "drehen: Drehgeschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehgeschwindigkeit
    assert type(dauer) is FloatType, "drehen: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer
    global anhalten 
    anhalten = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    twist = Twist()
    twist.linear.x = 0                   
    twist.linear.y = 0 
    twist.linear.z = 0          
    twist.angular.x = 0 
    twist.angular.y = 0   
    twist.angular.z = drehgeschwindigkeit
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not anhalten:
        publisher.publish(twist)   
        r.sleep()

def fahren(geschwindigkeit=0.2, dauer=1.0):
    assert type(geschwindigkeit) is FloatType, "fahren: Geschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % geschwindigkeit
    assert type(dauer) is FloatType, "fahren: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer
    global anhalten 
    anhalten = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    twist = Twist()
    twist.linear.x = geschwindigkeit                   
    twist.linear.y = 0 
    twist.linear.z = 0          
    twist.angular.x = 0 
    twist.angular.y = 0   
    twist.angular.z = 0
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not anhalten:
        publisher.publish(twist)   
        r.sleep()

def kurve(geschwindigkeit=0.2, drehgeschwindigkeit=1.0, dauer=1.0):
    assert type(drehgeschwindigkeit) is FloatType, "kurve: Drehgeschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehgeschwindigkeit
    assert type(geschwindigkeit) is FloatType, "kurve: Geschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % geschwindigkeit
    assert type(dauer) is FloatType, "kurve: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer
    global anhalten 
    anhalten = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    twist = Twist()
    twist.linear.x = geschwindigkeit                   
    twist.linear.y = 0 
    twist.linear.z = 0          
    twist.angular.x = 0 
    twist.angular.y = 0   
    twist.angular.z = drehgeschwindigkeit
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not anhalten:
        publisher.publish(twist)   
        r.sleep()

def on_odom(data):
    global odomdata
    odomdata=data
    global event
    event.set()
    global odomsubscriber
    odomsubscriber.unregister()
    

def positionwinkel():
    global odomsubscriber
    odomsubscriber = rospy.Subscriber("/odom", Odometry , on_odom)
    global event 
    event = threading.Event()
    event.wait()
    global odomdata
    x = odomdata.pose.pose.position.x
    y = odomdata.pose.pose.position.y
    quaternion = (odomdata.pose.pose.orientation.x, odomdata.pose.pose.orientation.y, odomdata.pose.pose.orientation.z, odomdata.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    return (x,y,yaw)

def anhalten():
    global anhalten
    anhalten = True
    twist = Twist()
    twist.linear.x = 0                   
    twist.linear.y = 0 
    twist.linear.z = 0          
    twist.angular.x = 0 
    twist.angular.y = 0   
    twist.angular.z = 0
    publisher.publish(twist)    

def dreheninrad(drehwinkelrad):
    assert type(drehwinkelrad) is FloatType, "dreheninrad: Drehwinkel %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehwinkelrad
    assert drehwinkelrad < 2 * math.pi, "dreheninrad: Drehwinkel %s muss kleiner als 2PI sein (einfache Drehung) " % drehwinkelrad
    global anhalten 
    anhalten = False
    x,y,yawstart = positionwinkel()
    if yawstart < 0.0:
        yawstart = math.pi + yawstart + math.pi 
    yawstop = (yawstart + drehwinkelrad) % (math.pi * 2.0)
    negativ = False
    if drehwinkelrad < 0.0:
        negativ = True
    delta = math.fabs(drehwinkelrad)
    twist = Twist()
    twist.linear.x = 0                   
    twist.linear.y = 0 
    twist.linear.z = 0          
    twist.angular.x = 0 
    twist.angular.y = 0   
    if negativ:
        twist.angular.z = -min(delta,1.0) 
    else:
        twist.angular.z = min(delta,1.0) 
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and delta > 0.01 and not anhalten:
        publisher.publish(twist)   
        r.sleep()
        x,y,yawaktuell = positionwinkel()
        if yawaktuell < 0.0:
            yawaktuell = math.pi + yawaktuell + math.pi 
        if negativ: 
            if yawaktuell > yawstop:
                delta = yawaktuell - yawstop
            else:
                delta = 2 * math.pi - yawstop + yawaktuell
            twist.angular.z = -min(delta,1.0) 
        else:
            if yawaktuell > yawstop:
                delta = 2 * math.pi - yawaktuell + yawstop
            else:
                delta = yawstop - yawaktuell
            twist.angular.z = min(delta,1.0) 
        print yawaktuell, yawstop, delta
    twist.angular.z = 0 
    publisher.publish(twist) 

def fahreninmeter(streckenlaenge):
    assert type(streckenlaenge) is FloatType, "fahren: streckenlaenge %s muss eine Gleitkommazahl sein (z.B. 1.0)" % streckenlaenge
    global anhalten 
    anhalten = False
    x,y,yaw = positionwinkel()
    zielx = math.cos(yaw) * streckenlaenge + x
    ziely = math.sin(yaw) * streckenlaenge + y
    yawstart = yaw
    if yawstart < 0.0:
        yawstart = math.pi + yawstart + math.pi
    deltabevor = streckenlaenge + 1 
    delta = math.fabs(streckenlaenge)
    twist = Twist()
    twist.linear.x = min(delta,1.0)                   
    twist.linear.y = 0 
    twist.linear.z = 0          
    twist.angular.x = 0 
    twist.angular.y = 0   
    twist.angular.z = 0
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown() and delta < deltabevor and not anhalten:
        publisher.publish(twist)   
        r.sleep()
        x,y,yaw = positionwinkel()  
        if yaw < 0.0:
            yaw = math.pi + yaw + math.pi   
        deltabevor = delta    
        delta = math.sqrt((zielx - x)**2 + (ziely - y)**2)
        if twist.linear.x > delta:
            twist.linear.x = delta
        if (yaw < yawstart and (yawstart - yaw) < math.pi):
            twist.angular.z = 0.05
        else: 
            twist.angular.z = -0.05
        print yawstart,yaw,delta
    twist.angular.z = 0 
    twist.linear.x = 0  
    publisher.publish(twist)

if __name__ == '__main__':
    rospy.init_node('bewegen', anonymous=True)
    print positionwinkel()
    fahreninmeter(1.0)
    dreheninrad(-math.pi/2)
    fahreninmeter(1.0)
    dreheninrad(-math.pi/2)
    fahreninmeter(1.0)
    dreheninrad(-math.pi/2)
    fahreninmeter(1.0)
    dreheninrad(-math.pi/2)
   # drehen(-0.75,2.0)
   # anhalten()
    
