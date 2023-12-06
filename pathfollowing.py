#!/usr/bin/env python
#
# Creation:
#   November 2023
#
#
# Description:
#   This code is intended for the implementation of simple path following algorithms for
#   testing purposes for DSOR newcommers

# ROS basics
import rospy
import numpy as np

# ROS messsages
from test_msgs.msg import mRef, mState

class PathFollowing:
    def __init__(self):
        rospy.loginfo('Initializing Path Following Node')
        rospy.init_node('pf_node')

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

        # Subscriber definition
        self.sub_state = rospy.Subscriber("/state", mState, self.stateCallback)

        # Publisher definition
        self.pub_ref = rospy.Publisher("/ref", mRef, queue_size=10)

        # Parameters definition
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.last_time = rospy.get_time()

    def stateCallback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.psi

    def pathCreation(self):
        a = 2
        t = self.last_time

        # points and derivatives of path
        pd = [a * np.sqrt(2) * np.cos(t) / (np.sin(t)**2 + 1),
           a * np.sqrt(2) * np.cos(t) * np.sin(t) / (np.sin(t)**2 + 1)]

        pd_dot = [- (2 * np.sqrt(2) * np.sin(t)) / (np.sin(t)**2 + 1) - (4 * sp.sqrt(2) * np.cos(t)**2 * np.sin(t)) / (np.sin(t)**2 + 1)**2,
                (2 * np.sqrt(2) * np.cos(t)**2) / (np.sin(t)**2 + 1) - (2 * np.sqrt(2) * np.sin(t)**2) / (np.sin(t)**2 + 1) - (4 * np.sqrt(2) * np.cos(t)**2 * np.sin(t)**2) / (np.sin(t)**2 + 1)**2]

        pd_ddot = [(16 * np.sqrt(2) * np.cos(t)**3 * np.sin(t)**2) / (np.sin(t)**2 + 1)**3 - (4 * np.sqrt(2) * np.cos(t)**3) / (np.sin(t)**2 + 1)**2 - (2 * np.sqrt(2) * np.cos(t)) / (np.sin(t)**2 + 1) + (12 * np.sqrt(2) * np.cos(t) * np.sin(t)**2) / (np.sin(t)**2 + 1)**2,
                (16 * np.sqrt(2) * np.cos(t)**3 * np.sin(t)**3) / (np.sin(t)**2 + 1)**3 - (8 * np.sqrt(2) * np.cos(t) * np.sin(t)) / (np.sin(t)**2 + 1) + (12 * np.sqrt(2) * np.cos(t) * np.sin(t)**3) / (np.sin(t)**2 + 1)**2 - (12 * np.sqrt(2) * np.cos(t)**3 * np.sin(t)) / (np.sin(t)**2 + 1)**2]

        # curvature
        k = (np.dot(np.array([1, 0]), pd_dot) * np.dot(np.array([0, 1]), np.cross(pd_ddot[0], pd_ddot[1]))) / (np.linalg.norm(pd_dot)**3)

        return pd, pd_dot, pd_ddot, k

    def timerCallback(self, event):
        new_time = rospy.get_time()
        dt = new_time - self.last_time

        pd, pd_dot, pd_ddot, k = self.pathCreation(self)

        # tunning knobs
        k1 = 0.11;
        k2 = 0.22;
        k3 = 0.33;
        theta = 0.2;
        k_delta = 1;

        p = np.array([self.x, self.y])
        psi = self.yaw

        psi_p = np.arctan2(np.dot(np.array([0, 1]), pd_dot), np.dot(np.array([1, 0]), pd_dot))

        if np.abs(self.psi_p_prev - psi_p) > np.pi:
            if self.psi_p_prev - psi_p > 0:
                psi_p = psi_p + 2 * np.pi
            elif self.psi_p_prev - psi_p < 0:
                psi_p = psi_p - 2 * np.pi

        # rotation matrix
        R = np.array([[np.cos(psi_p), np.sin(psi_p)], [-np.sin(psi_p), np.cos(psi_p)]])
        # error
        ep = np.dot(R, (p - pd))
        s1, y1 = ep[0], ep[1]
        psi_e = psi - psi_p

        # Speed input
        self.u = np.linalg.norm(pd_dot)
        u_dot = np.dot(pd_ddot.T, pd_dot) * gamma_dot / np.linalg.norm(pd_dot)
        up = u * np.cos(psi_e) + k3 * s1

        gamma_dot = up / np.linalg.norm(pd_dot)

        y1_dot = u * np.sin(psi_e) - s1 * k * up

        # delta e delta dot expressions
        delta = -theta * np.tanh(k_delta * y1 * u)
        delta_dot = -k_delta * theta * (1 - (y1_dot * u + y1 * u_dot) * np.tanh(k_delta * y1 * u)**2)
        
        psi_til = psi_e - delta
        self.r = k * up + delta_dot - k1 * psi_til - k2 * y1 * u * (np.sin(psi_e) - np.sin(delta)) / psi_til

        gamma = gamma + gamma_dot * dt;
        # publish
        ref_msg = mRef()
        ref_msg.x = self.u
        ref_msg.y = self.r
        self.pub_ref.publish(ref_msg)

        self.last_time = new_time

        pass

if __name__ == '__main__':
    pf_node = PathFollowing()