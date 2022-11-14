#!/usr/bin/env python2

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

import roslaunch
import rospy
import rospkg
import sys
from sensor_msgs.msg import PointCloud2

class MessageGetter(object):
    def __init__(self, topic, topic_type, timeout=1.0):
        self.topic = topic
        self.topic_type = topic_type
        self.timeout = timeout
    
    def get_message(self):
        result = None
        try:
            result = rospy.wait_for_message(self.topic, self.topic_type, self.timeout)
        except rospy.exceptions.ROSException as err:
            rospy.logerr("%s is not found", self.topic)
            rospy.logerr(err)
        else:
            rospy.loginfo("got %s correctly", self.topic)
        finally:
            return result

class Listener(object):
    def __init__(self, time_lilmit=10, topic_lanes='/points_lanes', topic_ground='/points_ground', msg_wait=1.0):
        self.topic_lanes_msg = MessageGetter(topic_lanes, PointCloud2, msg_wait)
        self.topic_ground_msg = MessageGetter(topic_ground, PointCloud2, msg_wait)
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
            msg_lanes = self.topic_lanes_msg.get_message()
            msg_ground = self.topic_ground_msg.get_message()
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