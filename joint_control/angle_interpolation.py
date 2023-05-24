'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import *


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        names, times, keys = keyframes
        curr_time = perception.time - self.start_time #time for interpolation
        
        for joint_i, j_name in enumerate(names):
            for key_frames in range(len(times[joint_i])-1):
                if (times[joint_i][key_frames] < curr_time < times[joint_i][len(times[joint_i])-1]):
                    target_joints[j_name] = self.bezier_calculation(joint_i, key_frames, curr_time, keyframes)

        if 'LHipYawPitch' in target_joints: #if Left Hip has a value then copy to right hip
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']

        return target_joints

    def bezier_calculation(self, joint_i, key_frames, curr_time, keyframes) :

        names, times, keys = keyframes

        t = (curr_time - times[joint_i][key_frames])/(times[joint_i][key_frames + 1] - times[joint_i][key_frames])
        p0 = keys[joint_i][key_frames][0]
        p1 = p0 + keys[joint_i][key_frames][2][2] #handle 2, point 1
        p3 = keys[joint_i][key_frames + 1][0] #handle 1, point 2
        p2 = p3 + keys[joint_i][key_frames + 1][1][2]
        return (p0*(1-t)**3 + p1*3*(1-t)**2*t + p2*3*(1-t)*t**2 + p3*t**3)

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()