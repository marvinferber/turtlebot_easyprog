#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import cv2
import threading
import math

from cv_bridge import CvBridge, CvBridgeError
import numpy 
from numpy import linalg

from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import ButtonEvent
from sensor_msgs.msg import Image

rot = numpy.array([255,0,0])
gruen = numpy.array([0,255,0])
blau = numpy.array([0,0,255])

def on_bumper(data):
    # Bumperevent
    if data.state==1: # Bumper pressed
        global bumperfunc
        bumperfunc()
    else: # Bumper released
        pass

def on_taste0(data):
    # Tastenevent
    if data.button==0: # Button0
        if data.state==1: # Pressed
            global b0func
            b0func()
        else: # Bumper released
            pass

def on_taste1(data):
    # Tastenevent
    if data.button==1: # Button1 
        if data.state==1: # Pressed
            global b1func
            b1func()
        else: # Bumper released
            pass

def on_taste2(data):
    # Tastenevent
    if data.button==2: # Button1 
        if data.state==1: # Pressed
            global b2func
            b2func()
        else: # Bumper released
            pass

def beibumpercrash(func):
    global bumperfunc
    bumperfunc = func
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, on_bumper)

def beitasteb0(func):
    global b0func
    b0func = func
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, on_taste0)

def beitasteb1(func):
    global b1func
    b1func = func
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, on_taste1)

def beitasteb2(func):
    global b2func
    b2func = func
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, on_taste2)

def test():
    print "test called"     

def on_img(data):
    global bilderhalten
    if bilderhalten == True:
        return
    bridge = CvBridge()
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    bilderhalten = True
    global event
    event.set()

def farbeistin0(farbe,pixel=5000):
    return farbeistin(farbe,0,160,0,213,pixel)

def farbeistin1(farbe,pixel=5000):
    return farbeistin(farbe,0,160,214,426,pixel)

def farbeistin2(farbe,pixel=5000):
    return farbeistin(farbe,0,160,427,640,pixel)

def farbeistin3(farbe,pixel=5000):
    return farbeistin(farbe,161,320,0,213,pixel)

def farbeistin4(farbe,pixel=5000):
    return farbeistin(farbe,161,320,214,426,pixel)

def farbeistin5(farbe,pixel=5000):
    return farbeistin(farbe,161,320,427,640,pixel)

def farbeistin6(farbe,pixel=5000):
    return farbeistin(farbe,321,480,0,213,pixel)

def farbeistin7(farbe,pixel=5000):
    return farbeistin(farbe,321,480,214,426,pixel)

def farbeistin8(farbe,pixel=5000):
    return farbeistin(farbe,321,480,427,640,pixel)

def farbeistin(farbe, startrow, endrow, startcol, endcol, pixel):
    if type(farbe) == 'str':
        if farbe == "rot":
            vergleichsfarbe = rot
        elif farbe == "grün":
            vergleichsfarbe = gruen
        elif farbe == "blau":
            vergleichsfarbe = blau
        else:
            raise ValueError(farbe+" ist keine bekannte Farbe! [rot, grün, blau]")
    else:
        vergleichsfarbe = numpy.array([farbe[0],farbe[1],farbe[2]])
    global bilderhalten
    bilderhalten = True
    global event
    event = threading.Event()
    global imgsubscriber
    if (not 'imgsubscriber' in globals()) or (imgsubscriber == False):
        imgsubscriber = True
        rospy.Subscriber("/camera/rgb/image_raw", Image, on_img)
        print "Initialisiere Kamera für 2 Sekunden.."
        rospy.sleep(2.)
    bilderhalten = False
    event.wait()
    # Farben und Bereiche prüfen
    global cv_image
    count = 0
    (rows,cols,channels) = cv_image.shape
    neuesbild = numpy.zeros([endrow-startrow, endcol-startcol, channels], numpy.uint8) 
    for i in range(startrow,endrow):
        for j in range(startcol,endcol):
            if linalg.norm(cv_image[i,j] - vergleichsfarbe)<50:
                neuesbild[i-startrow,j-startcol] = [vergleichsfarbe[0],vergleichsfarbe[1],vergleichsfarbe[2]]#[255,255,255]
                count = count+1
            else:
                neuesbild[i-startrow,j-startcol] = cv_image[i,j]#[0,0,0]
    cv2.imshow(`startrow+startcol`, neuesbild)
    cv2.waitKey(3)
    print count
    if count > pixel :
        return True
    else:
        return False

def farbhistogramm():
    global bilderhalten
    bilderhalten = True
    global event
    event = threading.Event()
    global imgsubscriber
    if (not 'imgsubscriber' in globals()) or (imgsubscriber == False):
        imgsubscriber = True
        rospy.Subscriber("/camera/rgb/image_raw", Image, on_img)
        print "Initialisiere Kamera für 2 Sekunden.."
        rospy.sleep(2.)
    bilderhalten = False
    event.wait()
    # Farben und Bereiche prüfen
    global cv_image
    (rows,cols,channels) = cv_image.shape
    gr=gb=gg=0
    for i in range(161,320):
        for j in range(214,426):
            (r,g,b)=cv_image[i,j]
            gr=gr+r
            gg=gg+g
            gb=gb+b
    gesamt=(320-161)*(426-214)
    gr=gr/gesamt
    gg=gg/gesamt
    gb=gb/gesamt
    return (gr,gg,gb)
    

if __name__ == '__main__':
    rospy.init_node('sensoren', anonymous=True)
#    beibumpercrash(test)
#    beitasteb0(test)
#    beitasteb1(test)
#    beitasteb2(test)
#    farbeistin0("rot")
    my = farbhistogramm()
    print farbeistin0(my)
    print farbeistin1(my)
    print farbeistin2(my)
    print farbeistin3(my)
    print farbeistin4(my)
    print farbeistin5(my)
    print farbeistin6(my)
    print farbeistin7(my)
    print farbeistin8(my)

#   print farbeistin4(my)
#    print farbeistin4(my)
    rospy.spin()


