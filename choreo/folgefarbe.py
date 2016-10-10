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

from turtlebot_easyprog import sensoren
from turtlebot_easyprog import bewegen
from turtlebot_easyprog import aktoren

rospy.init_node('folgefarbe', anonymous=True)
# Bitte nachfolgend Code
sensoren.beibumpercrash(bewegen.anhalten)
folgefarbe = sensoren.farbhistogramm()
bewegen.drehen(1.0,4.0)
while True:
    if sensoren.farbeistin4(folgefarbe):#mitte
        bewegen.fahren(0.1,2.0)
    elif sensoren.farbeistin3(folgefarbe): #links
        bewegen.drehen(0.5)
        bewegen.fahren(0.1,2.0)
    elif sensoren.farbeistin5(folgefarbe): #rechts
        bewegen.drehen(-0.5)
        bewegen.fahren(0.1,2.0)
#######
rospy.spin()
