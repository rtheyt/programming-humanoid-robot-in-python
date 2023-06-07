'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref

#imports needed
import xmlrpc.client
import threading
from threading import Thread
import numpy as np

from keyframes import *

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        list_keyframes = [keyframes]
        Thread(target = self.serv.execute_keyframes, args = list_keyframes).start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        list_st = [effector_name, transform]
        Thread(target = self.serv.set_transform, args = list_st).start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.serv = xmlrpc.client.ServerProxy('http://localhost:9999', allow_none=True)
        #test if client is launched
        print('client launched')

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.serv.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.serv.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.serv.get_posture()
    
    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.serv.execute_keyframes(keyframes)
    
    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.serv.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.serv.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE


    agent.set_angle('LElbowYaw', 3)
    
    print(agent.get_angle('LElbowYaw'))

    #test with transfo matrix

    transfo_matrix = np.eye(4)
    transfo_matrix[2,3] = 0.1
    #agent.set_transform('Lleg', transfo_matrix)

