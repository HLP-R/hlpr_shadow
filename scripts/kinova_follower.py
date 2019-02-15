#/usr/bin/env python

from hlpr_shadow.shadow_follower import *
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
from sensor_msgs.msg import JointState
import rospy

class KinovaFollower(ShadowFollower):
    JNT_NAMES = ['j2s7s300_joint_'+str(i+1) for i in range(7)]

    def _joint_state_dict(self, joint_state_list):
        return {name: state for name, state in zip(self.JNT_NAMES, joint_state_list)}

    def _init_robot(self):
        self.arm_moveit_ = ArmMoveIt()
        self.gripper_ = Gripper()
        self.gripper_.open()
        self.pub_ = rospy.Publisher(self.joint_ctrl_topic, JointState, queue_size=1)

    def _goto_home(self):
        joint_state = self._joint_state_dict([3.0857096802238977,
                                              2.867071611579444,
                                              -0.030040457096093806,
                                              5.413631253335634,
                                              6.252699036917013,
                                              2.6120238697758635,
                                              4.86700146063033])
        self.arm_moveit_.move_to_joint_pose(joint_state)

    def _goto(self, joint_state):
        self.pub_.publish(joint_state)

    def _open_gripper(self):
        self.gripper_.open()

    def _close_gripper(self):
        self.gripper_.close()

def main():
    follower = KinovaFollower('/kinova_jogger/target',
                              '/j2s7s300_driver/out/joint_state',
                              host='10.145.212.232', port=8000)
    follower.run()

if __name__ == '__main__':
    main()
        
