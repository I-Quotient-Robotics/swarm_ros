#!/usr/bin/env python

import time
import tf
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from swarm_msgs.msg import SwarmCommand
from swarm_msgs.srv import SetCommand, SetCommandResponse

swarm_command = 'dismiss'

def set_swarm_command_cb(req):
    global swarm_command

    rospy.loginfo('Change Command %s to %s', swarm_command, req.command)
    swarm_command = req.command
    return SetCommandResponse('ok')

def main():
    global swarm_command

    rospy.init_node('swarm_center')
    swarm_command_pub = rospy.Publisher('swarm_command', SwarmCommand, queue_size=10)
    set_command_service = rospy.Service('swarm_set_type', SetCommand, set_swarm_command_cb)

    tf_listener = tf.TransformListener()

    frame_id = rospy.get_param('~frame_id', 'base_footprint')

    base_pose = PoseStamped()
    base_pose.header.frame_id = frame_id

    swarm_command_msg = SwarmCommand()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        swarm_command_msg.header.stamp = rospy.Time.now()
        swarm_command_msg.command = swarm_command

        pose = PoseStamped()
        pose.header.frame_id = frame_id

        try:
            # base_pose.header.stamp = rospy.Time.now()
            pose = tf_listener.transformPose('map', base_pose)
            swarm_command_msg.pose = pose.pose
            swarm_command_pub.publish(swarm_command_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('tf error, %s' % e)

        rate.sleep()

if __name__ == '__main__':
    main()
