#!/usr/bin/env python

import time
import rospy
import roslibpy

from actionlib import SimpleActionClient
from actionlib.action_server import ActionServer

from geometry_msgs.msg import Pose, Twist, PoseStamped

ros_client = roslibpy.Ros('127.0.0.1', 9090)

swarm_pose = roslibpy.Topic(ros_client, 'swarm_pose', 'swarm_msgs/SwarmPose')
swarm_command = roslibpy.Topic(ros_client, 'swarm_command', 'swarm_msgs/SwarmCommand')
swarm_heartbeat = roslibpy.Topic(ros_client, 'swarm_hearbeat', 'swarm_msgs/SwarmHeartbeat')
swarm_base_pose = roslibpy.Topic(ros_client, 'swarm_base_pose', 'swarm_msgs/SwarmPose')

def command_callback(message):
    rospy.loginfo("command_callback get data")
# 
def base_pose_callback(message):
    rospy.loginfo("base_pose_callback get data")

def start_sending():
    rospy.loginfo('start sending')
    while True:
        if not ros_client.is_connected:
            break
        # swarm_pose.publish(roslibpy.Message({'command': 'hello world'}))

        heartbeat_msg = {}
        heartbeat_msg['name'] = 'robot_1'
        heartbeat_msg['state'] = 'alone'
        pose_msg = {}
        pose_msg['position'] = {'x':1.0, 'y':2.0, 'z':3.0}
        pose_msg['orientation'] = {'x':1.0, 'y':2.0, 'z':3.0, 'w':4.0}
        heartbeat_msg['pose'] = pose_msg
        swarm_heartbeat.publish(roslibpy.Message(heartbeat_msg))
        time.sleep(1)

def start_receiving():
    rospy.loginfo('start receiving')
    swarm_command.subscribe(command_callback)
    swarm_base_pose.subscribe(base_pose_callback)

def main():
    rospy.init_node("swarm_client")

    ros_client.on_ready(start_receiving, run_in_thread=True)
    ros_client.on_ready(start_sending, run_in_thread=True)

    # ros_client.run_forever()
    while not rospy.is_shutdown():
        ros_client.run()

    swarm_command.unsubscribe()
    swarm_base_pose.unadvertise()
    ros_client.call_later(2, ros_client.terminate)

    rospy.loginfo("shutdown....ok")

if __name__ == '__main__':
    main()