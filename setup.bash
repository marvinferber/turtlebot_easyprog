#!/usr/bin/env bash

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

# aliases to start turtlebot stuff
alias turtle_dance='roslaunch turtlebot_easyprog dance.launch'
alias turtle_view='roslaunch turtlebot_rviz_launchers view_robot.launch'
alias turtle_sim='roslaunch turtlebot_easyprog turtlesim.launch'
alias turtle_key='rosrun turtlesim turtle_teleop_key'
alias turtle_control='export ROS_MASTER_URI="http://127.0.0.1:11311" && export ROS_HOSTNAME="127.0.0.1" && roslaunch turtlebot_easyprog xbox360.launch'
alias turtle_robot='export ROS_MASTER_URI="http://$MASTER:11311" && export ROS_HOSTNAME="$MASTER" &&  roslaunch turtlebot_easyprog minimal.launch'
alias turtle_set_local='export ROS_MASTER_URI="http://127.0.0.1:11311" && export ROS_HOSTNAME="127.0.0.1"'
alias turtle_set_remote='export ROS_MASTER_URI="http://$MASTER:11311" && export ROS_IP="$MYIP"'
alias turtle_gazebo='roslaunch turtlebot_gazebo turtlebot_world.launch'

export TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/indigo/share/turtlebot_gazebo/worlds/empty.world

