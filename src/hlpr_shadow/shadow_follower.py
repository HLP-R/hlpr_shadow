#!/usr/bin/env python

from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Lock
import signal
import rospy

class ShadowFollower(object):
    JOINT_MOVE_THRESH = float('inf')

    def __init__(self, joint_ctrl_topic, joint_state_topic, 
                    host='localhost', port=8000, name='shadow_follower'):
        self.joint_ctrl_topic = joint_ctrl_topic
        self.joint_state_topic = joint_state_topic
        self.host = host
        self.port = port
        self.name = name
        self.js_lock = Lock()
        self.joint_state = None
        self.following = True

    def _init_robot(self):
        raise NotImplementedError('ShadowFollower._init_robot not implemented.')
    
    def _goto_home(self):
        raise NotImplementedError('ShadowFollower._goto_home not implemented.')

    def _goto(self, joint_state):
        raise NotImplementedError('ShadowFollower._goto not implemented.') 

    def _open_gripper(self):
        raise NotImplementedError('ShadowFollower._open_gripper not implemented.')
    
    def _close_gripper(self):
        raise NotImplementedError('ShadowFollower._close_gripper not implemented.')

    def _update_joint_state(self, msg):
        with self.js_lock:
            self.joint_state = msg.position

    def _shadow_joints(self, header, name, position, velocity, effort):
        with self.js_lock:
            matches = [abs(self.joint_state[i]-position[i]) < self.JOINT_MOVE_THRESH
                        for i in range(len(self.joint_state))]

        if all(matches):
            self.following = True
            js = JointState(Header(header['seq'], rospy.Time.from_sec(header['stamp']), header['frame_id']),
                            name, position, velocity, effort)
            self._goto(js)
        elif self.following:
            self.following = False
            rospy.logerr('Follower has drifted too far from leader.')

    def run(self):
        rospy.init_node(self.name)

        self._init_robot()
        self._goto_home()

        # subscribe to joint states
        self.joint_state = rospy.wait_for_message(self.joint_state_topic, JointState)
        rospy.Subscriber(self.joint_state_topic, JointState, self._update_joint_state)

        # create server
        server = SimpleXMLRPCServer((self.host, self.port), logRequests=False, allow_none=True)
        server.register_introspection_functions()

        # register shadow_joints function
        server.register_function(self._shadow_joints, 'shadow_joints')

        # register gripper functions
        server.register_function(self._open_gripper, 'open_gripper')
        server.register_function(self._close_gripper, 'close_gripper')

        while not rospy.is_shutdown():
            server.handle_request()

