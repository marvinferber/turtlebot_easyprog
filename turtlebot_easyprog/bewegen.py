#!/usr/bin/env python

# Copyright 2016 Marvin Ferber
#
# Licensed under the Apache License, Version 2.0 (the "License");
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
    twist.linear.x = 0;                   
    twist.linear.y = 0; 
    twist.linear.z = 0;          
    twist.angular.x = 0; 
    twist.angular.y = 0;   
    twist.angular.z = drehgeschwindigkeit;
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
    twist.linear.x = geschwindigkeit;                   
    twist.linear.y = 0; 
    twist.linear.z = 0;          
    twist.angular.x = 0; 
    twist.angular.y = 0;   
    twist.angular.z = 0;
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
    twist.linear.x = geschwindigkeit;                   
    twist.linear.y = 0; 
    twist.linear.z = 0;          
    twist.angular.x = 0; 
    twist.angular.y = 0;   
    twist.angular.z = drehgeschwindigkeit;
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
    twist.linear.x = 0;                   
    twist.linear.y = 0; 
    twist.linear.z = 0;          
    twist.angular.x = 0; 
    twist.angular.y = 0;   
    twist.angular.z = 0;
    publisher.publish(twist)       

if __name__ == '__main__':
    rospy.init_node('bewegen', anonymous=True)
    print positionwinkel()
   # drehen(-0.75,2.0)
   # anhalten()
    
