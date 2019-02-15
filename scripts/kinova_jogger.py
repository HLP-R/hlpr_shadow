#/usr/bin/env python

from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointVelocity
from threading import Lock
import numpy as np
import rospy

class KinovaJogger(object):
    JNT_NAMES = ['j2s7s300_joint_'+str(i+1) for i in range(7)]
    RATE = 100
    K_p = 255.
    K_v = 10.

    def __init__(self, joint_target_topic='/kinova_jogger/target',
                    joint_state_topic='/j2s7s300_driver/out/joint_state',
                    joint_vel_topic='/j2s7s300_driver/in/joint_velocity',
                    name='kinova_jogger'):
        self.joint_target_topic = joint_target_topic
        self.joint_state_topic = joint_state_topic
        self.joint_vel_topic = joint_vel_topic
        self.name = name
        self.target_pos = None
        self.target_vel = None
        self.current_pos = None
        self.current_vel = None
        self.t_lock = Lock()
        self.c_lock = Lock()

    def _joint_state_dict(self, name, position):
        return {n: p for n, p in zip(name, position)}

    def _simplify_angles(self, angles):
        prev_rev = 2*np.pi*np.floor(angles/(2*np.pi))
        next_rev = 2*np.pi*np.ceil(angles/(2*np.pi))
        s_angles = []
        for i in range(len(angles)):
            if np.fabs(angles[i]-prev_rev[i]) < np.fabs(angles[i]-next_rev[i]):
                s_angles.append(angles[i]-prev_rev[i])
            else:
                s_angles.append(angles[i]-next_rev[i])
        return np.array(s_angles)

    def _nearest_equivalent(self, target, current):
        # compute current number of revolutions
        prev_rev = np.floor(current/(2*np.pi))
        next_rev = np.ceil(current/(2*np.pi))
        curr_rev = np.zeros(len(current))
        for i in range(len(current)):
            if np.fabs(current[i] - 2*np.pi*prev_rev[i]) < np.fabs(current[i] - 2*np.pi*next_rev[i]):
                curr_rev[i] = prev_rev[i]
            else:
                curr_rev[i] = next_rev[i]

        # compute closest angle
        low_val = 2*np.pi*(curr_rev - 1) + target
        med_val = 2*np.pi*curr_rev + target
        high_val = 2*np.pi*(curr_rev + 1) + target
        n_eq = np.zeros(len(current))
        for i in range(len(current)):
            if np.fabs(current[i] - low_val[i]) <= np.fabs(current[i] - med_val[i]) \
                and np.fabs(current[i] - low_val[i]) <= np.fabs(current[i] - high_val[i]):
                n_eq[i] = low_val[i]

            elif np.fabs(current[i] - med_val[i]) <= np.fabs(current[i] - low_val[i]) \
                    and np.fabs(current[i] - med_val[i]) <= np.fabs(current[i] - high_val[i]):
                n_eq[i] = med_val[i]

            else:
                n_eq[i] = high_val[i]

        return n_eq

    def set_target(self, msg):
        joint_state = self._joint_state_dict(msg.name, msg.position)
        joint_velocity = self._joint_state_dict(msg.name, msg.velocity)
        with self.t_lock:
            self.target_pos = self._simplify_angles(np.array([joint_state[j] for j in self.JNT_NAMES]))
            self.target_vel = self._simplify_angles(np.array([joint_velocity[j] for j in self.JNT_NAMES]))

    def set_current(self, msg):
        joint_state = self._joint_state_dict(msg.name, msg.position)
        joint_velocity = self._joint_state_dict(msg.name, msg.velocity)
        with self.c_lock:
            self.current_pos = self._simplify_angles(np.array([joint_state[j] for j in self.JNT_NAMES]))
            self.current_vel = self._simplify_angles(np.array([joint_velocity[j] for j in self.JNT_NAMES]))

    def run(self):
        rospy.init_node(self.name)

        rospy.Subscriber(self.joint_target_topic, JointState, self.set_target)
        rospy.Subscriber(self.joint_state_topic, JointState, self.set_current)
        pub = rospy.Publisher(self.joint_vel_topic, JointVelocity, queue_size=1)

        rate = rospy.Rate(self.RATE)
        error_pos = np.zeros(7)
        error_vel = np.zeros(7)
        while not rospy.is_shutdown():
            with self.t_lock and self.c_lock:
                if not self.target_pos is None and not self.current_pos is None:
                    error_pos = self._nearest_equivalent(self.target_pos, self.current_pos) - self.current_pos
                    error_vel = self._nearest_equivalent(self.target_vel, self.current_vel) - self.current_vel

            vels = self.K_p*error_pos + self.K_v*error_vel

            vel_msg = JointVelocity()
            vel_msg.joint1 = vels[0]
            vel_msg.joint2 = vels[1]
            vel_msg.joint3 = vels[2]
            vel_msg.joint4 = vels[3]
            vel_msg.joint5 = vels[4]
            vel_msg.joint6 = vels[5]
            vel_msg.joint7 = vels[6]

            pub.publish(vel_msg)
            rate.sleep()

if __name__ == '__main__':
    jogger = KinovaJogger()
    jogger.run()

