#!/usr/bin/env python3
 
# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import sys
import json
import argparse

import rospy
from std_msgs.msg import String
from map_mux.msg import MapRequest




def main(argv = sys.argv):

    default_fleet_name = 'fleet_name'
    default_robot_name = 'robot_name'
    default_map_number = 1
    default_level_name = "level_0"
    default_topic_name = 'map_request'
    

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=default_fleet_name)
    parser.add_argument('-r', '--robot-name', default=default_robot_name)
    parser.add_argument('-i', '--map-number', default=default_map_number)
    parser.add_argument('-n', '--level-name', default=default_level_name)
    parser.add_argument('-t', '--topic-name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('fleet_name: {}'.format(args.fleet_name))
    print('robot_name: {}'.format(args.robot_name))
    print('map_number: {}'.format(args.map_number))
    print('map_number: {}'.format(args.level_name))
    print('topic_name: {}'.format(args.topic_name))

    rospy.init_node('map_request_node', anonymous=True)
    pub = rospy.Publisher(args.topic_name, MapRequest,  queue_size=10)

    msg = MapRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name
    msg.map_number = int(args.map_number)
    msg.level_name = args.level_name

    if (not rospy.is_shutdown()):
        rospy.loginfo("publishing map request")
        rospy.loginfo("fleet_name: "+ msg.fleet_name)
        rospy.loginfo("robot_name: "+ msg.robot_name)
        rospy.loginfo("robot_name: "+ msg.level_name)
        rospy.loginfo("map_number: "+ str(msg.map_number))
        rospy.sleep(1)
        
        pub.publish(msg)
        print('all done!')


if __name__ == '__main__':
    main(sys.argv)
