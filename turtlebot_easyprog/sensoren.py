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

from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import ButtonEvent


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

if __name__ == '__main__':
    rospy.init_node('sensoren', anonymous=True)
    beibumpercrash(test)
    beitasteb0(test)
    beitasteb1(test)
    beitasteb2(test)
    rospy.spin()


