#!/usr/bin/env python2

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

import roslaunch
import rospy
import rospkg
import sys
from sensor_msgs.msg import PointCloud2

class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1.0):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message

class Listener(object):
    def __init__(self, time_lilmit=10, topic_lanes='/points_lanes', topic_ground='/points_ground', msg_wait=1.0):
        self.topic_lanes_msg = SensorMessageGetter(topic_lanes, PointCloud2, msg_wait)
        self.topic_ground_msg = SensorMessageGetter(topic_ground, PointCloud2, msg_wait)
        self.time_lilmit = time_lilmit
        self.end_time = None

    def is_time_lilmit(self):
        if self.time_lilmit is None or self.end_time is None:
            return False
        return rospy.Time.now() >= self.end_time
    
    def test_node(self):
        self.end_time = None
        if self.time_lilmit is not None:
            self.end_time = rospy.Time.now() + rospy.Duration.from_sec(self.time_lilmit)
        while self.is_time_lilmit() is False:
            msg_lanes = self.topic_lanes_msg.get_msg()
            msg_ground = self.topic_ground_msg.get_msg()
            if msg_lanes is not None and msg_ground is not None:
                return True
        return False

def test_node():

    rospy.init_node('test_node', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    r = rospkg.RosPack()
    p = r.get_path('ground_filter')
    path = p + "/launch/test.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path])

    launch.start() # Launch test
    rospy.loginfo("Started")

    node = Listener()
    result = node.test_node()

    launch.shutdown()
    if result:
        rospy.loginfo("Success")
        sys.exit(0)
    else:
        rospy.loginfo("Fail")
        sys.exit(1)
    

if __name__ == '__main__':
    test_node() 