'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from autograd import grad
from scipy.optimize import fmin
from math import atan2

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        
        # YOUR CODE HERE
        
        length_vec = len(self.chains[effector_name])
        joint_names = self.chains[effector_name]
        
        end_effector_name = self.chains[effector_name][length_vec-1]
        joint_angles = {key : self.perception.joint[key] 
                        for key in self.chains[effector_name]} 
        
        #get the goal vector (target)
        goal_vec = np.matrix(self.get_goal_trans(transform))
        
        #from optimization.ipynb¨
        func = lambda t: self.error_func(t, goal_vec, end_effector_name)

        optimization = fmin(func, joint_angles)
        joint_angles_dict = dict(zip(joint_names, optimization))


        return joint_angles_dict

    def get_goal_trans(self, T_matrix) :
        
        angle_x = 0
        angle_y = 0
        angle_z = 0

        x = T_matrix[3,0]
        y = T_matrix[3,1]
        z = T_matrix[3,2]
    
        #chose between the rot matrixes
        if(T_matrix[0,0] == 1):       
            angle_x= np.atan2(T_matrix[2,1], T_matrix[1,1]) 
        elif(T_matrix[1,1] == 1):      
            angle_y = np.atan2(T_matrix[0,2], T_matrix[0,0])
        elif(T_matrix[2,2] == 1):
            angle_z = np.atan2(T_matrix[0,1], T_matrix[0,0])  

        goal = np.array([x,y,z,angle_x,angle_y,angle_z])
        return  goal
    
    def error_func(self, joint_angles, target, ee_name):
        self.forward_kinematics[joint_angles]
        intermatrix = self.transforms[ee_name] #we get the total multiplied matrix
        final_vec = np.matrix(self.get_goal_trans(intermatrix))
        err = target -  final_vec
        return np.linalg.norm(err)           

     

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angle_dict = self.inverse_kinematics(effector_name, transform)
        names = self.chains(effector_name)
        times = [0, 3] * len(self.chains[effector_name])
        for name in enumerate(names):
            keys = [[self.perception.joint[name], [3,0,0], [3,0,0]], [angle_dict[name], [3,0,0], [3,0,0]]]
        self.keyframes = (names, times, keys)
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
