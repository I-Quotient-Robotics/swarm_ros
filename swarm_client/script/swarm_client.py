#!/usr/bin/env python

import time
import PyKDL

import tf
import rospy
import roslibpy
from tf_conversions import posemath

from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseFeedback, MoveBaseAction

class SwarmClient():
    def __init__(self):
        # base robot ip and port
        self.__base_port = rospy.get_param('~base_port', 9090)
        self.__base_ip = rospy.get_param('~base_address', '192.168.31.16')

        # robot name, each robot should have different name
        self.__robot_name = rospy.get_param('~robot_name', 'scout-mini-02')
        self.__robot_frame_id = rospy.get_param('~robot_frame_id', 'base_footprint')

        self.__load_swarm_trans()
        self.__current_command = 'dismiss'

        self.__tf_listener = tf.TransformListener()

        self.__ros_client = roslibpy.Ros(self.__base_ip, self.__base_port)

        self.__swarm_command = roslibpy.Topic(self.__ros_client, 'swarm_command', 'swarm_msgs/SwarmCommand')
        self.__swarm_heartbeat = roslibpy.Topic(self.__ros_client, 'swarm_hearbeat', 'swarm_msgs/SwarmHeartbeat')

        # self.__move_base_pub = rospy.Publisher('swarm_goal', PoseStamped, queue_size=30)
        self.__move_base_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=30)

        self.__target_pose = PoseStamped()
        self.__swarm_base_pose = Pose()

        self.__ros_client.on_ready(self.__start_sending, run_in_thread=True)
        self.__ros_client.on_ready(self.__start_receiving, run_in_thread=True)

        self.__ros_client.run()

        rospy.loginfo('Swarm client started')
    
    def target_pose(self):
        return self.__target_pose

    def swarm_base_pose(self):
        return self.__swarm_base_pose

    def __load_swarm_trans(self):
        type_list = rospy.get_param('swarm_trans')

        self.__swarm_trans = {}
        for name in type_list:
            data = rospy.get_param('swarm_trans/'+name)
            self.__swarm_trans[name] = PyKDL.Frame(PyKDL.Rotation.RPY(data[3], data[4], data[5]), PyKDL.Vector(data[0], data[1], data[2]))

        rospy.loginfo("get %d swarm type: %s", len(type_list), type_list)

    def __command_callback(self, message):
        if not self.__swarm_trans.has_key(message['command']):
            if message['command'] != 'dismiss':
                rospy.logwarn('unknown command type %s', message['command'])
                return

        self.__current_command = message['command']

        self.__swarm_base_pose.position.x = message['pose']['position']['x']
        self.__swarm_base_pose.position.y = message['pose']['position']['y']
        self.__swarm_base_pose.position.z = message['pose']['position']['z']
        self.__swarm_base_pose.orientation.x = message['pose']['orientation']['x']
        self.__swarm_base_pose.orientation.y = message['pose']['orientation']['y']
        self.__swarm_base_pose.orientation.z = message['pose']['orientation']['z']
        self.__swarm_base_pose.orientation.w = message['pose']['orientation']['w']

        p = posemath.fromMsg(self.__swarm_base_pose)

        self.__target_pose.header.stamp = rospy.Time.now()
        self.__target_pose.header.frame_id = 'map'
        self.__target_pose.pose = posemath.toMsg(p*self.__swarm_trans[self.__current_command])
        self.__move_base_pub.publish(self.__target_pose)

    def __start_sending(self):
        rospy.loginfo('start sending')

        # send robot hearbeat msg to swarm_base robot
        rate = rospy.Rate(2)
        while self.__ros_client.is_connected:
            try:
                (trans, orient) = self.__tf_listener.lookupTransform('map', self.__robot_frame_id, rospy.Time(0))
                pose_msg = {}
                pose_msg['position'] = {'x':trans[0], 'y':trans[1], 'z':trans[2]}
                pose_msg['orientation'] = {'x':orient[0], 'y':orient[1], 'z':orient[2], 'w':orient[3]}

                heartbeat_msg = {}
                heartbeat_msg['header'] = {}
                heartbeat_msg['header']['stamp'] = rospy.Time.now()
                heartbeat_msg['header']['frame_id'] = self.__robot_name
                heartbeat_msg['state'] = self.__current_command
                heartbeat_msg['pose'] = pose_msg

                self.__swarm_heartbeat.publish(roslibpy.Message(heartbeat_msg))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn('tf 1error, %s' % e)

            # rospy.loginfo('Swarm State: command %s', self.__current_command)
            rate.sleep()

    def __start_receiving(self):
        rospy.loginfo('start receiving')
        self.__swarm_command.subscribe(self.__command_callback)

    def run(self):
        pass

    def close(self):
        self.__swarm_command.unsubscribe()
        self.__ros_client.call_later(2, self.__ros_client.terminate)

def main():
    rospy.init_node('swarm_client')

    swarm_client = SwarmClient()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()

    rospy.loginfo('shutdown....')
    swarm_client.close()
    rospy.loginfo('ok')

if __name__ == '__main__':
    main()
