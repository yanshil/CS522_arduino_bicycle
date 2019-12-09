import os
import math
import numpy as np

import gym
from gym import spaces
from gym.utils import seeding

import pybullet as p
import pybullet_data

class BalancebotEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self, render=False):
        self.targetPosition = (-5,4)
        self._observation = []
        self.action_space = spaces.Discrete(5)
        ## Observation_space (3+3+1): Position(x,y,z) + Orientation(eulerX,eulerY,eulerZ) + back_wheel_velocity(1)
        self.observation_space = spaces.Box(np.array([-math.inf, -math.inf, -1, -math.pi, -math.pi, -math.pi  , -500]), 
                                            np.array([-math.inf, -math.inf, 1, math.pi, math.pi, math.pi  , 500])) # pitch, gyro, com.sp.

        if (render):
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF

        self._seed()
        
        # paramId = p.addUserDebugParameter("My Param", 0, 100, 50)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        self._assign_throttle(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()
        info = self._get_info()

        self._envStepCounter += 1

        return np.array(self._observation), reward, done, info

    def _reset(self):
        # reset is called once at initialization of simulation
        ## Back Wheel Velocity
        self.vt = -200
        #self.vd = 0
        self.maxV = 300 # 235RPM = 24,609142453 rad/sec
        self._envStepCounter = 0

        ## Bar Position
        self.pb = 0
        self.maxP = math.pi / 2.0

        p.resetSimulation()
        p.setGravity(0,0,-10) # m/s^2
        p.setTimeStep(0.01) # sec
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "BricsCAD_Bicycle/BricsCAD_Bicycle.urdf"),
                           cubeStartPos,
                           cubeStartOrientation)

        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def _assign_throttle(self, action):
        deltav = 0
        deltaA = 0
        # 0: do nothing
        if action == -1 or action == 0:
            pass
        # 1: Accelerate
        elif action == 1:
            deltav = -25 #(minus is forward...)
        # 2: Decelerate
        elif action == 2:
            deltav = 25
        # 3: Turn Left
        elif action == 3:
            deltaA = -2
        # 4: Turn Right
        elif action == 4:
            deltaA = 2

        vt = clamp(self.vt + deltav, -self.maxV, self.maxV)
        self.vt = vt
        pb = clamp(self.pb + deltaA, -self.maxP, self.maxP)
        self.pb = pb

        ## Set position control for the bar
        p.setJointMotorControl2(bodyUniqueId = self.botId, 
                                jointIndex=0, 
                                controlMode=p.POSITION_CONTROL, 
                                targetPosition=pb, 
                                force= 240.)
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=2, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=vt)


    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        return [cubePos[0], cubePos[1], cubePos[2],cubeEuler[0],cubeEuler[1],cubeEuler[2],self.vt]

    # def _compute_reward(self):
    #     return 0.1 - abs(self.vt - self.vd) * 0.005
    def _compute_reward(self):
        cubePos, _ = p.getBasePositionAndOrientation(self.botId)
        #cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        return -((self.targetPosition[0]-cubePos[0])**2 + (self.targetPosition[1] - cubePos[1])**2)

    def _get_info(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        info = {
            'cubePos': cubePos,
            'cubeEuler': cubeEuler,
            'back_wheel_velocity': self.vt
        }
        return info

    def _compute_done(self):
        cubePos, _ = p.getBasePositionAndOrientation(self.botId)
        ## Fall below 12 degree
        return cubePos[2] < 0.766 or cubePos[2] > 0.83 or self._envStepCounter >= 1500

    def _render(self, mode='human', close=False):
        pass

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
