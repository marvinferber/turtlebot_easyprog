#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Marc Donner
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
import numpy as np
import tf.transformations as tft
from turtlesim.msg import Pose
from geometry_msgs.msg import Quaternion,Vector3
from nav_msgs.msg import Odometry

BASE_MSG = Odometry()
BASE_MSG.pose.covariance = (np.eye(6).reshape(36) * 1e-5).tolist()
BASE_MSG.twist.covariance = (np.eye(6).reshape(36) * 1e-5).tolist()
BASE_MSG.header.frame_id = "odom"
BASE_MSG.child_frame_id = "base_link"


def on_pose(turtlesim_pose):
    global BASE_MSG
    # turtlesim_pose = Pose()
    BASE_MSG.pose.pose.position.x = turtlesim_pose.x
    BASE_MSG.pose.pose.position.y = turtlesim_pose.y
    BASE_MSG.pose.pose.orientation = Quaternion(*tft.quaternion_from_euler(0, 0, turtlesim_pose.theta))

    BASE_MSG.twist.twist.angular = Vector3(0, 0, turtlesim_pose.angular_velocity)
    BASE_MSG.twist.twist.linear = Vector3(turtlesim_pose.linear_velocity, 0, 0)

    BASE_MSG.header.seq += 1
    BASE_MSG.header.stamp = rospy.get_rostime()
    pub.publish(BASE_MSG)
    max_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("turtlesim_odom", anonymous=True)
    max_rate = rospy.Rate(25)

    sub = rospy.Subscriber("/turtle1/pose", Pose, on_pose, queue_size=3)
    pub = rospy.Publisher("/odom",Odometry,queue_size=3)

    rospy.spin()
    pass
