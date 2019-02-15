#!/usr/bin/env python

from hlpr_shadow.shadow_leader import *
from robotiq_85_msgs.msg import GripperStat
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
from collections import deque
import rospy

class KinovaLeader(ShadowLeader):
    def __init__(self, joint_state_topic, gripper_state_topic,
                    follower_uris=[('localhost',8000)], name='shadow_leader'):
        self.gripper_state_topic = gripper_state_topic
        self.gripper_pos_history_ = deque([], 2)
        super(KinovaLeader, self).__init__(joint_state_topic, follower_uris, name)

    def _joint_state_dict(self, joint_state_list):
        names = ['j2s7s300_joint_'+str(i+1) for i in range(7)]
        return {name: state for name, state in zip(names, joint_state_list)}

    def _init_robot(self):
        self.arm_ = ArmMoveIt()
        self.gripper_ = Gripper()
        self.gripper_.open()
        rospy.Subscriber(self.gripper_state_topic, GripperStat, self._set_grip_state)

    def _goto_home(self):
        joint_state = self._joint_state_dict([3.0857096802238977,
                                              2.867071611579444,
                                              -0.030040457096093806,
                                              5.413631253335634,
                                              6.252699036917013,
                                              2.6120238697758635,
                                              4.86700146063033])
        self.arm_.move_to_joint_pose(joint_state)

    def _set_grip_state(self, msg):
        self.gripper_pos_history_.append(msg.position)
        if len(self.gripper_pos_history_) > 1:
            delta = self.gripper_pos_history_[1] - self.gripper_pos_history_[0]
            if abs(delta) > 0:
                self.gripper_open_ = delta > 0

def main():
    leader = KinovaLeader('/j2s7s300_driver/out/joint_state', '/gripper/stat',
                          follower_uris=[('10.145.90.132',8000)])
    leader.run()

if __name__ == '__main__':
    main()

