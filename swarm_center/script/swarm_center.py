#!/usr/bin/env python

import time
import tf
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped

def main():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('swarm_center')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    main()