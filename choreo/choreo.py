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

#from turtlebot_easyprog import sensoren
import turtlebot_easyprog.bewegen as bewegen
#from turtlebot_easyprog import aktoren

rospy.init_node('choreo', anonymous=True)
# Bitte nachfolgend Code
#sensoren.beibumpercrash(bewegen.anhalten)
#bewegen.fahren(0.1,2.0)
print bewegen.positionwinkel()
bewegen.drehen(-0.5,1.0)
print bewegen.positionwinkel()
#bewegen.drehen(-0.5,1.0)
#bewegen.kurve(0.5,0.2,2)
#sensoren.beitasteb0(aktoren.led1)
#######
rospy.spin()
