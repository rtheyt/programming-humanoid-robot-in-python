'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

#needed imports
import threading
from threading import Thread
from xmlrpc.server import SimpleXMLRPCServer


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def __init__(self):
        super(ServerAgent, self).__init__()
        self.serv = SimpleXMLRPCServer(('localhost', 9999),logRequests=True, allow_none=True) #logrequest =1 to verifiy the connection in terminal

        self.serv.register_function(self.get_angle)
        self.serv.register_function(self.set_angle)
        self.serv.register_function(self.get_posture)
        self.serv.register_function(self.execute_keyframes)
        self.serv.register_function(self.get_transform)
        self.serv.register_function(self.set_transform)

        list_args_thread = [] #thread needs lists
        target_thread = self.serv.serve_forever
        self.thread_forever = Thread(target=target_thread, args = list_args_thread)
        self.thread_forever.start()

        #test 
        print('server launched')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint.get(joint_name)

        
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        self.start_time = self.perception_time 

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

