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
        # base robot ip and port
        self.__base_port = 9090
        self.__base_ip = '192.168.0.2'

        # robot name, each robot should have different name
        self.__robot_name = "robot_1"
        self.__current_command = "dismiss"

        # follow position
        self.__follow_translation = (-1.5, 1.0, 0.0)
        self.__follow_rotation = (0.0, 0.0, 0.0, 1.0)

        # cover position
        self.__cover_translation = (1.5, 0.0, 0.0)
        self.__cover_rotation = (0.0, 0.0, 0.0, 1.0)

        # dismiss position
        self.__dismiss_ready = False
        self.__dismiss_translation = (0.0, 1.5, 0.0)
        self.__dismiss_rotation = (0.0, 0.0, 0.0, 1.0)

        self.__tf_listener = tf.TransformListener()
        self.__tf_broadcaster = tf.TransformBroadcaster()

        self.__ros_client = roslibpy.Ros(self.__base_ip, self.__base_port)

        self.__swarm_command = roslibpy.Topic(self.__ros_client, 'swarm_command', 'std_msgs/String')
        self.__swarm_heartbeat = roslibpy.Topic(self.__ros_client, 'swarm_hearbeat', 'swarm_msgs/SwarmHeartbeat')
        self.__swarm_base_pose = roslibpy.Topic(self.__ros_client, 'swarm_base_pose', 'geometry_msgs/PoseStamped')

        # self.__movebase_client = SimpleActionClient('move_base', MoveBaseAction)
        # rospy.loginfo("wait for movebase server...")
        # self.__movebase_client.wait_for_server()
        # rospy.loginfo("movebase server connected")

        self.__move_base_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=30)

        self.__ros_client.on_ready(self.__start_sending, run_in_thread=True)
        self.__ros_client.on_ready(self.__start_receiving, run_in_thread=True)

    def __command_callback(self, message):
        # rospy.loginfo("command_callback get data: %s", message['data'])
        self.__current_command = message['data']

    def __base_pose_callback(self, message):
        # rospy.loginfo("base_pose_callback get data")

        # publish swarm_base robot tf on map
        translation = (message['pose']['position']['x'], message['pose']['position']['y'], message['pose']['position']['z'])
        rotation = (message['pose']['orientation']['x'], message['pose']['orientation']['y'], message['pose']['orientation']['z'], message['pose']['orientation']['w'])
        self.__tf_broadcaster.sendTransform(translation,
                                            rotation,
                                            rospy.Time.now(),
                                            "swarm_base", "map")

        # publish follow pose tf
        self.__tf_broadcaster.sendTransform(self.__follow_translation,
                                            self.__follow_rotation,
                                            rospy.Time.now(),
                                            "swarm_follow", "swarm_base")

        # pbulish cover pose tf
        self.__tf_broadcaster.sendTransform(self.__cover_translation,
                                            self.__cover_rotation,
                                            rospy.Time.now(),
                                            "swarm_cover", "swarm_base")

        # pbulish dismiss pose tf
        self.__tf_broadcaster.sendTransform(self.__dismiss_translation,
                                            self.__dismiss_rotation,
                                            rospy.Time.now(),
                                            "swarm_cover", "swarm_dismiss")

        relative_pose = PoseStamped()
        # relative_pose.header.stamp = rospy.Time.now()
        if(self.__current_command=="follow"):
            relative_pose.header.frame_id = "swarm_follow"
        elif(self.__current_command=="cover"):
            relative_pose.header.frame_id = "swarm_cover"
        elif(self.__current_command=="dismiss"):
            relative_pose.header.frame_id = "swarm_dismiss"

        # get pose, based on map frame
        try:
            target_pose = self.__tf_listener.transformPose("map", relative_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("tf 1error, %s" % e)

        # send new goal to movebase
        if(self.__current_command=="dismiss" and self.__dismiss_ready==False):
            # dismiss goal send only one time, so set __dismiss_ready flag,
            # after send movebase goal
            self.__dismiss_ready = True
            self.__move_base_pub.publish(target_pose)
        elif(self.__current_command != "dismiss"):
            # rospy.loginfo("move")
            # mb_goal = MoveBaseGoal()
            # mb_goal.target_pose = target_pose
            # self.__movebase_client.send_goal(mb_goal)
            self.__dismiss_ready = False
            self.__move_base_pub.publish(target_pose)

    def __start_sending(self):
        rospy.loginfo('start sending')

        # send robot hearbeat msg to swarm_base robot
        rate = rospy.Rate(2)
        while self.__ros_client.is_connected:
            heartbeat_msg = {}
            heartbeat_msg['name'] = self.__robot_name
            heartbeat_msg['state'] = self.__current_command
            try:
                (trans, orient) = self.__tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                pose_msg = {}
                pose_msg['position'] = {'x':trans[0], 'y':trans[1], 'z':trans[2]}
                pose_msg['orientation'] = {'x':orient[0], 'y':orient[1], 'z':orient[2], 'w':orient[3]}
                heartbeat_msg['pose'] = pose_msg
                self.__swarm_heartbeat.publish(roslibpy.Message(heartbeat_msg))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("tf 1error, %s" % e)

            rospy.loginfo("Swarm State: command %s", self.__swarm_command)
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
