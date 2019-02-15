#!/usr/bin/env python

from sensor_msgs.msg import JointState
from collections import deque
import xmlrpclib
import socket
import rospy

class ShadowLeader(object):
    def __init__(self, joint_state_topic, follower_uris=[('localhost',8000)], name='shadow_leader'):
        self.joint_state_topic = joint_state_topic
        self.follower_uris = follower_uris
        self.name = name
        self.followers_ = None
        self.followers_online_ = []
        self.grip_state_history_ = deque([True, True], 2)
        self.gripper_open_ = True

    def _init_robot(self):
        raise NotImplementedError('ShadowLeader._init_robot not implemented.')

    def _goto_home(self):
        raise NotImplementedError('ShadowLeader._goto_home not implemented.')

    def _set_grip_state(self, msg):
        raise NotImplementedError('ShadowLeader._set_grip_state not implemented.')

    def _get_uri(self, host, port):
        return 'http://'+host+':'+str(port)

    def _wait_for_followers(self):
        self.followers_online_ = [False for host, port in self.follower_uris]

        while not all(self.followers_online_):
            try:
                for idx, follower in enumerate(self.followers_):
                    follower.system.listMethods()
                    self.followers_online_[idx] = True
                rospy.loginfo('Connected to followers!')
            except socket.error:
                rospy.loginfo('Waiting for followers...')
                rospy.sleep(1)

    def _move(self, msg):
        header = {'seq': msg.header.seq,
                  'stamp': msg.header.stamp.to_sec(),
                  'frame_id': msg.header.frame_id}
        self.grip_state_history_.append(self.gripper_open_)

        for idx, follower in enumerate(self.followers_):
            try:
                follower.shadow_joints(header, msg.name, msg.position, msg.velocity, msg.effort)

                if self.grip_state_history_[0] != self.grip_state_history_[1]:
                    if self.grip_state_history_[1]:
                        follower.open_gripper()
                    else:
                        follower.close_gripper()

                if not self.followers_online_[idx]:
                    rospy.loginfo('Connection to follower at ' 
                                     + self._get_uri(*self.follower_uris[idx])
                                     + ' reestablished.')
                self.followers_online_[idx] = True
            except socket.error:
                if self.followers_online_[idx]:
                    rospy.logerr('Connection to follower at ' 
                                    + self._get_uri(*self.follower_uris[idx]) + ' lost.')
                self.followers_online_[idx] = False

    def run(self):
        rospy.init_node(self.name)
        
        self._init_robot()
        self._goto_home()
       
        # set up followers 
        self.followers_ = [xmlrpclib.ServerProxy(self._get_uri(host, port), allow_none=True) 
                            for host, port in self.follower_uris]
        self._wait_for_followers()

        # subscribe to joint states
        rospy.Subscriber(self.joint_state_topic, JointState, self._move)
        rospy.spin()

