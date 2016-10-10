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

from kobuki_msgs.msg import Led
#from types import *

publisher = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

def led1(farbe = "rot"):
    led=Led()
    if farbe == "rot":
        led.value = 3 # red Led
    elif farbe == "grün":
        led.value = 1 # green Led
    elif farbe == "test":
        led.value = 2
    else:
        raise ValueError(farbe+" ist keine Farbe!")
    print led.value
    publisher.publish(led)

if __name__ == '__main__':
    rospy.init_node('aktoren', anonymous=True)
    led1("grün")
    led1("grün")
    led1("grün")
    rospy.spin()
