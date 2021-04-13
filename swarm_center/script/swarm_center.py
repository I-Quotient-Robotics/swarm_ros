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
    swarm_command_pub = rospy.Publisher('swarm_base_pose', SwarmCommand, queue_size=10)
    set_command_service = rospy.Service('swarm_set_command', SetCommand, set_swarm_command_cb)

    tf_listener = tf.TransformListener()

    frame_id = rospy.get_param('~frame_id', 'base_footprint')

    base_pose = PoseStamped()
    base_pose.header.frame_id = frame_id

    SwarmCommand swarm_command_msg

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():

        swarm_command_msg.header.stamp = ros.Time.now()
        swarm_command_msg.command = swarm_command

        pose = PoseStamped()
        pose.header.frame_id = frame_id

        try:
            pose = tf_listener.transformPose('map', base_pose)
            swarm_command_msg.pose = pose.pose
            set_command_service.publish(swarm_command_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('tf error, %s' % e)

        rate.sleep()

if __name__ == '__main__':
    main()
