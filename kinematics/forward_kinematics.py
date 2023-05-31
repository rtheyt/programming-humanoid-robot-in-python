'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
import numpy as np
from numpy.matlib import matrix, identity

from numpy import sin, cos

from recognize_posture import PostureRecognitionAgent



class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                        'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                        'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']   
                       }
        self.link_translation = {'HeadYaw': [0., 0., 0.1265], 'HeadPitch':	[0., 0., 0.], 
                                 'RShoulderPitch':	[0., -0.098, 0.1], 'RShoulderRoll': [0., 0.,	0.], 'RElbowYaw':	[0.105, -0.015, 0.], 'RElbowRoll':[0., 0., 0.],
                                 'LShoulderPitch':	[0., 0.098, 0.1],'LShoulderRoll': [0., 0.,	0.], 'LElbowYaw':	[0.105, 0.015, 0.], 'LElbowRoll':[0., 0., 0.], 
                                 'RHipYawPitch': [0, -0.05, -0.085], 'RHipRoll': [0., 0., 0.], 'RHipPitch': [0., 0., 0.],'RKneePitch': [0., 0., -0.1], 'RAnklePitch': [0., 0., -0.1029], 'RAnkleRoll': [0., 0., 0.],
                                 'LHipYawPitch': [0., 0.05, -0.085], 'LHipRoll': [0., 0., 0.], 'LHipPitch': [0., 0., 0.], 'LKneePitch': [0., 0., -0.1], 'LAnklePitch': [0., 0., -0.1029], 'LAnkleRoll': [0., 0., 0.]
                                }
    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        
        # YOUR CODE HERE
        c1 = cos(joint_angle)
        s1 = sin(joint_angle)
        Tr_matrix = identity(4)
        if 'Roll' in joint_name:              
            Rot_matrix = matrix([[1, 0, 0], [0,c1,-s1], [0, s1, c1]])  
        elif 'Pitch' in joint_name:   
            Rot_matrix = matrix([[c1, 0, s1], [0, 1, 0], [-s1, 0, c1]]) 
        elif 'Yaw' in joint_name:
            Rot_matrix = matrix([[c1, s1, 0], [-s1, c1, 0], [0, 0, 1]])

        Tr_matrix[0:3, 0:3] = Rot_matrix

        Tr_matrix[3,0] = self.link_translation[joint_name][0]  
        Tr_matrix[3,1] = self.link_translation[joint_name][1]
        Tr_matrix[3,2] = self.link_translation[joint_name][2]

        return Tr_matrix

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            Tr_matrix = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                T_1 = self.local_trans(joint, angle)
                # YOUR CODE HERE
                Tr_matrix = Tr_matrix*T_1
                self.transforms[joint] = Tr_matrix

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
