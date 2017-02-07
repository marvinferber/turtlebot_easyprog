#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Marvin Ferber, Marc Donner
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
from Queue import Queue
import tf
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from types import *


class global_state:
    odom_queue = Queue(maxsize=1)
    odom_sub = None
    twist_msg = Twist()


def on_odom(data):
    global_state.odom_queue.put(data)
    # global_state.odom_sub.unregister()


publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
global_state.odom_sub = rospy.Subscriber("/odom", Odometry, on_odom, queue_size=1)


def drehen(drehgeschwindigkeit=1.0, dauer=1.0):
    assert type(
        drehgeschwindigkeit) is FloatType, "drehen: Drehgeschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehgeschwindigkeit
    assert type(dauer) is FloatType, "drehen: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer
    global stop
    stop = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = drehgeschwindigkeit
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not stop:
        publisher.publish(twist)
        r.sleep()


def fahren(geschwindigkeit=0.2, dauer=1.0):
    assert type(
        geschwindigkeit) is FloatType, "fahren: Geschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % geschwindigkeit
    assert type(dauer) is FloatType, "fahren: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer
    global stop
    stop = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    twist = Twist()
    twist.linear.x = geschwindigkeit
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not stop:
        publisher.publish(twist)
        r.sleep()


def kurve(geschwindigkeit=0.2, drehgeschwindigkeit=1.0, dauer=1.0):
    assert type(
        drehgeschwindigkeit) is FloatType, "kurve: Drehgeschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehgeschwindigkeit
    assert type(
        geschwindigkeit) is FloatType, "kurve: Geschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % geschwindigkeit
    assert type(dauer) is FloatType, "kurve: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer
    global stop
    stop = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    twist = Twist()
    twist.linear.x = geschwindigkeit
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = drehgeschwindigkeit
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not stop:
        publisher.publish(twist)
        r.sleep()


def kurvegeschmeidig(geschwindigkeit=0.2, beschleunigung=0.05, drehgeschwindigkeit=1.0, drehbeschleunigung=0.5,
                     dauer=1.0):
    assert type(
        drehgeschwindigkeit) is FloatType, "kurve: Drehgeschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehgeschwindigkeit
    assert type(
        geschwindigkeit) is FloatType, "kurve: Geschwindigkeit %s muss eine Gleitkommazahl sein (z.B. 1.0)" % geschwindigkeit
    assert type(dauer) is FloatType, "kurve: Dauer %s muss eine Gleitkommazahl sein (z.B. 1.0)" % dauer

    global stop
    stop = False
    start = rospy.get_rostime()
    dauer = rospy.Duration.from_sec(dauer)
    control_speed = 0
    control_turn = 0
    twist = Twist()
    twist.linear.x = control_speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = control_turn
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and ((start + dauer) > rospy.get_rostime()) and not stop:
        # stepwise TODO fix as sequence of commands
        if geschwindigkeit > control_speed:
            control_speed = min(geschwindigkeit, control_speed + beschleunigung)
        elif geschwindigkeit < control_speed:
            control_speed = max(geschwindigkeit, control_speed - beschleunigung)
        else:
            control_speed = geschwindigkeit
        # stepwise TODO fix as sequence of commands
        if drehgeschwindigkeit > control_turn:
            control_turn = min(drehgeschwindigkeit, control_turn + drehbeschleunigung)
        elif drehgeschwindigkeit < control_turn:
            control_turn = max(drehgeschwindigkeit, control_turn - drehbeschleunigung)
        else:
            control_turn = drehgeschwindigkeit
        twist.linear.x = control_speed
        twist.angular.z = control_turn
        publisher.publish(twist)
        r.sleep()


def positionwinkel():
    # global_state.odom_sub = rospy.Subscriber("/odom", Odometry, on_odom)
    _ = global_state.odom_queue.get() # possibly old data
    cur_odom = global_state.odom_queue.get()

    x = cur_odom.pose.pose.position.x
    y = cur_odom.pose.pose.position.y
    quaternion = (cur_odom.pose.pose.orientation.x, cur_odom.pose.pose.orientation.y,
                  cur_odom.pose.pose.orientation.z, cur_odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    return (x, y, yaw)


def anhalten():
    global stop
    stop = True
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    publisher.publish(twist)


PI2 = math.pi * 2


def angle_to_0_2pi(a):
    return (a + PI2) % PI2


def angle_to_pi_pi(a):
    a2 = angle_to_0_2pi(a)
    return a2 if a2 <= math.pi else a2 - PI2


def pos_delta(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2.0 + (y1 - y2) ** 2.0)

def compute_delta(pose1, pose2):
    p = pos_delta(pose1[0], pose1[1], pose2[0], pose2[1])
    r = angle_delta(pose1[2],pose2[2])
    return p,r


def angle_delta(a1, a2, direction=0):
    """
      both input angles, a1 and a2, must be in [-pi,pi]
      return a difference d between two angles (in radians), so a1 + d = a2 or d = a2 - a1.
      when direction is 0, the smallest distance will be returned, always in range [-pi,pi].
      when direction is > 0, d will always be positive, always in range [0,2pi]
      when direction is < 0, d will always be negative, always in range [-2pi,0]

      examples:
        angle_delta(0.5,0.6,0) -> 0.1
        angle_delta(0.5,0.6,-1) -> -(2pi - 0.1)

    """
    delta = angle_to_pi_pi(a2 - a1)
    if direction == 0:
        return delta
    if direction > 0:
        return angle_to_0_2pi(delta + PI2)
    if direction < 0:
        return -angle_to_0_2pi(PI2 - delta)


def dreheninrad(drehwinkelrad):
    assert type(
        drehwinkelrad) is FloatType, "dreheninrad: Drehwinkel %s muss eine Gleitkommazahl sein (z.B. 1.0)" % drehwinkelrad
    assert drehwinkelrad < 2 * math.pi, "dreheninrad: Drehwinkel %s muss kleiner als 2PI sein (einfache Drehung) " % drehwinkelrad
    global stop
    stop = False

    _, _, yaw_aktuell = positionwinkel()
    yaw_ziel = angle_to_pi_pi(yaw_aktuell + drehwinkelrad)

    print yaw_ziel
    delta_davor = abs(drehwinkelrad) +1
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and not stop:
        _, _, yaw_aktuell = positionwinkel()

        delta = abs(angle_delta(yaw_aktuell,yaw_ziel, drehwinkelrad))


        if delta > delta_davor:
            break
        # print yaw_todo, yaw_davor
        print "delta:", delta
        global_state.twist_msg.angular.z = np.sign(drehwinkelrad) * max(0.1, min(1.5, delta))

        publisher.publish(global_state.twist_msg)
        delta_davor = delta
        r.sleep()

    global_state.twist_msg.angular.z = 0
    publisher.publish(global_state.twist_msg)


def fahreninmeter(streckenlaenge):
    assert type(
        streckenlaenge) is FloatType, "fahren: streckenlaenge %s muss eine Gleitkommazahl sein (z.B. 1.0)" % streckenlaenge
    global stop
    stop = False

    x, y, yaw = positionwinkel()

    print "c", x, y, yaw

    zielx = math.cos(yaw) * streckenlaenge + x
    ziely = math.sin(yaw) * streckenlaenge + y
    yawstart = yaw

    print "z", zielx, ziely

    fahrstrecke = 0
    twist = Twist()
    twist.linear.z = 0.2
    publisher.publish(twist)

    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and not stop:
        xalt = x
        yalt = y
        x, y, yaw = positionwinkel()
        fahrstrecke = fahrstrecke + pos_delta(x, y, xalt, yalt)
        print fahrstrecke
        if fahrstrecke > abs(streckenlaenge):
            break
        delta_yaw = angle_delta(yaw, yawstart)
        # korrekturfaktor abhängig von rate
        delta_yaw = delta_yaw * r.sleep_dur.to_sec()

        if abs(delta_yaw) > 0.05:
            twist.angular.z = delta_yaw
        else:
            twist.angular.z = 0

        twist.linear.x = 0.1 * np.sign(streckenlaenge)
        publisher.publish(twist)
        r.sleep()

    print "done", x, y, yaw
    # anhalten
    twist.angular.z = 0
    twist.linear.x = 0
    publisher.publish(twist)

def fahrenstrecke(streckenlaenge, winkel):
    assert type(
        streckenlaenge) is FloatType, "fahren: streckenlaenge %s muss eine Gleitkommazahl sein (z.B. 1.0)" % streckenlaenge
    global stop
    stop = False

    x, y, yaw = positionwinkel()

    print "c", x, y, yaw

    yawstart = yaw + winkel

    fahrstrecke = 0
    twist = Twist()
    
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and not stop:
        xalt = x
        yalt = y
        x, y, yaw = positionwinkel()
        fahrstrecke = fahrstrecke + pos_delta(x, y, xalt, yalt)
        #print fahrstrecke
        if fahrstrecke > abs(streckenlaenge):
            break
        delta_yaw = angle_delta(yaw, yawstart)
        # korrekturfaktor 
        delta_yaw = delta_yaw * 2
        print delta_yaw

        if abs(delta_yaw) > 0.05:
            twist.angular.z = delta_yaw
        else:
            twist.angular.z = 0

        twist.linear.x = 0.2 * np.sign(streckenlaenge)
        publisher.publish(twist)
        r.sleep()

    print "done", x, y, yaw
    # anhalten
    #twist.angular.z = 0
    #twist.linear.x = 0
    #publisher.publish(twist)

def fahrenposition(zielx, ziely):
    """
     Ermöglicht das Ansteuern einer absoluten Position. Die aktuelle Position wird über 
     positionwinkel() bestimmt und entsprechend wird die Zielposition in diesem 
     Koordinatensystem angefahren. (Aktuell /odom --> Drift über Fahrstrecke)
    """
    assert type(zielx) is FloatType, "zielx %s muss eine Gleitkommazahl sein (z.B. 1.0)" % zielx
    assert type(ziely) is FloatType, "ziely %s muss eine Gleitkommazahl sein (z.B. 1.0)" % ziely    
    global stop
    stop = False

    x, y, yaw = positionwinkel()

    print "c", x, y, yaw

    print "z", zielx, ziely

    delta = pos_delta(zielx, ziely, x, y)
    delta_davor = delta + 1
    twist = Twist()
    
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown() and not stop:
        x, y, yaw = positionwinkel()
        delta = pos_delta(zielx, ziely, x, y)
        # Abbruch der Beweung, wenn nahe am Ziel und sich wieder entfernt
        if (delta < 0.1) and (delta > delta_davor):
            break
        # winkel beta der Strecke Roboter --> Zielpunkte (Unterscheidung 4 Quadranten)
        beta = np.sign(ziely -y) *  (math.pi - math.atan(abs((ziely -y)/(zielx - x))) if (zielx-x)<0 else math.atan(abs((ziely -y)/(zielx - x))))
        # winkel zwischen beta und Roboterorientierung 
        delta_yaw = angle_delta( yaw, beta)
        print delta_yaw, delta, beta
        # nahe am Ziel Drehgeschwindigkeit deaktivieren, um Starke Rotation zu vermeiden
        if abs(delta_yaw) > 0.05 and (delta > 0.1):
            twist.angular.z = delta_yaw
        else:
            twist.angular.z = 0
        # max 0.2 und min 0.1 m/s 
        twist.linear.x = max(min(0.2, delta),0.1)
        publisher.publish(twist)
        delta_davor = delta + 0.0001
        r.sleep()

    print "done", delta, delta_davor
    print "done", x, y, yaw

def PointsInCircum(r,n=100):
    return [(math.cos(2*math.pi/n*x)*r,math.sin(2*math.pi/n*x)*r) for x in xrange(0,n+1)]    

if __name__ == '__main__':
    rospy.init_node('bewegen', anonymous=True)
    publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1) 
    global_state.odom_sub = rospy.Subscriber("/odom", Odometry, on_odom, queue_size=1)
    for (x,y) in PointsInCircum(1.0,20):
        fahrenposition(x,y)
#    fahreninmeter(1.0)
#    dreheninrad(-math.pi / 2)
#    fahreninmeter(1.0)
#    dreheninrad(-math.pi / 2)
#    fahreninmeter(1.0)
#    dreheninrad(-math.pi / 2)
#    fahrenposition(1.5, math.pi/4)
#    fahrenposition(1.0, 1.0)
#    fahrenposition(1.0, 2.0)
#    fahrenposition(0.0, 3.0)
#    fahrenposition(0.0, 1.0)
#    fahrenposition(0.0, 0.0)
#    fahrenposition(0.1, -math.pi/8)
#    fahrenposition(0.1, -math.pi/8)
#    fahrenposition(0.1, -math.pi/8)
#    dreheninrad(-math.pi / 2)
    # drehen(-0.75,2.0)
    # stop()
