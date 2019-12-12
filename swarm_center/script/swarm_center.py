#!/usr/bin/env python

import time
import tf
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped

from swarm_msgs.srv import SetCommand, SetCommandResponse

swarm_command = "dismiss"

def set_swarm_command_cb(req):
    global swarm_command
    swarm_command = req.command
    rospy.loginfo("Get Command: %s", req.command)
    return SetCommandResponse('ok')

def main():
    global swarm_command
    swarm_base_pub = rospy.Publisher('swarm_base_pose', PoseStamped, queue_size=10)
    swarm_command_pub = rospy.Publisher('swarm_command', String, queue_size=10)
    rospy.init_node('swarm_center')
    set_command_service = rospy.Service('swarm_set_command', SetCommand, set_swarm_command_cb)

    tf_listener = tf.TransformListener()

    base_pose = PoseStamped()
    base_pose.header.frame_id = "base_link"

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        pose = PoseStamped()
        pose.header.frame_id = "base_link"

        try:
            pose = tf_listener.transformPose("map", base_pose)
            pose.header.stamp = rospy.Time.now()
            swarm_base_pub.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("tf 1error, %s" % e)

        swarm_command_pub.publish(swarm_command)
        rate.sleep()

if __name__ == '__main__':
    main()