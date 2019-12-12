#!/usr/bin/env python

import time
import rospy
import tf
import roslibpy

from actionlib import SimpleActionClient

from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseFeedback, MoveBaseAction

class SwarmClient():
    def __init__(self):
        self.__current_command = "dismiss"

        self.__tf_listener = tf.TransformListener()
        self.__tf_broadcaster = tf.TransformBroadcaster()

        self.__ros_client = roslibpy.Ros('127.0.0.1', 9090)

        self.__swarm_command = roslibpy.Topic(self.__ros_client, 'swarm_command', 'std_msgs/String')
        self.__swarm_heartbeat = roslibpy.Topic(self.__ros_client, 'swarm_hearbeat', 'swarm_msgs/SwarmHeartbeat')
        self.__swarm_base_pose = roslibpy.Topic(self.__ros_client, 'swarm_base_pose', 'geometry_msgs/PoseStamped')

        self.__movebase_client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("wait for movebase server...")
        self.__movebase_client.wait_for_server()
        rospy.loginfo("movebase server connected")

        self.__ros_client.on_ready(self.__start_sending, run_in_thread=True)
        self.__ros_client.on_ready(self.__start_receiving, run_in_thread=True)

    def __command_callback(self, message):
        rospy.loginfo("command_callback get data: %s", message['data'])
        self.__current_command = message['data']

    def __base_pose_callback(self, message):
        rospy.loginfo("base_pose_callback get data")

        # publish swarm_base robot tf on map
        translation = (message['pose']['position']['x'], message['pose']['position']['y'], message['pose']['position']['z'])
        rotation = (message['pose']['orientation']['x'], message['pose']['orientation']['y'], message['pose']['orientation']['z'], message['pose']['orientation']['w'])
        dock_tf_broadcaster.sendTransform(  translation,
                                            rotation,
                                            rospy.Time.now(),
                                            "swarm_base", "map")

        # follow pose, based on base-robot pose
        relative_pose = PoseStamped()
        relative_pose.header.stamp = rospy.Time.now()
        relative_pose.header.frame_id = "swarm_base"
        relative_pose.pose.position.x = -1.0
        relative_pose.pose.position.y = -1.0
        relative_pose.pose.position.z = 0.0

        # get follow pose, based on map frame
        try:
            target_pose = self.__tf_listener.transformPose("map", relative_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.__dock_ready_pose_2.pose.position.z = -1.0
            rospy.logwarn("tf 1error, %s" % e)

        # send new goal to movebase action server
        if(self.__current_command == "follow"):
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose = target_pose
            # self.__movebase_client.send_goal(mb_goal)
        else if(self.__current_command == "cover"):
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose = target_pose
            # self.__movebase_client.send_goal(mb_goal)

    def __start_sending(self):
        rospy.loginfo('start sending')

        rate = rospy.Rate(1)
        while self.__ros_client.is_connected:
            heartbeat_msg = {}
            heartbeat_msg['name'] = 'robot_1'
            heartbeat_msg['state'] = 'alone'
            try:
                (trans, orient) = self.__tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                pose_msg = {}
                pose_msg['position'] = {'x':trans[0], 'y':trans[1], 'z':trans[2]}
                pose_msg['orientation'] = {'x':orient[0], 'y':orient[1], 'z':orient[2], 'w':orient[3]}
                heartbeat_msg['pose'] = pose_msg
                self.__swarm_heartbeat.publish(roslibpy.Message(heartbeat_msg))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("tf 1error, %s" % e)
            rate.sleep()

    def __start_receiving(self):
        rospy.loginfo('start receiving')
        self.__swarm_command.subscribe(self.__command_callback)
        self.__swarm_base_pose.subscribe(self.__base_pose_callback)

    def run(self):
        self.__ros_client.run()

    def close(self):
        self.__swarm_command.unsubscribe()
        self.__swarm_base_pose.unadvertise()
        self.__ros_client.call_later(2, self.__ros_client.terminate)

def main():
    rospy.init_node("swarm_client")

    swarm_client = SwarmClient()

    while not rospy.is_shutdown():
        swarm_client.run()

    swarm_client.close()

    rospy.loginfo("shutdown....ok")

if __name__ == '__main__':
    main()