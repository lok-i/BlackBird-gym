import pybullet_data
import pybullet
import time
import numpy as np
from gym.utils import seeding
from gym import spaces
import gym
import math
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


class blackBirdEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, renders=False, realWorld = 0):
        self.renders = renders
        if (renders):
            pybullet.connect(pybullet.GUI)
        else:
            pybullet.connect(pybullet.DIRECT)

        action_high = np.array([1, 1, 1, 1, 1, 1, 1, 1])
        self.action_space = spaces.Box(low=-action_high, high=action_high)
        self.observation_space = spaces.Box(low=-2, high=2, shape=(12,))
        self.seed(int(time.time()))
        self.dog = pybullet.loadURDF(
            currentdir+"/urdf/blackbird.urdf", [0, 0, 2], flags=pybullet.URDF_USE_SELF_COLLISION)
        pybullet.resetBasePositionAndOrientation(
            self.dog, [0, 0, 1.2], [0, 0, 0.707, 0.707])
        self.plane = pybullet.loadURDF(os.path.join(
            pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
    
        pybullet.changeDynamics(self.plane, -1, lateralFriction = 25)

        self.numJoints = pybullet.getNumJoints(self.dog)
        self.timeStep = 1.0/240
        self.currentSimTime = 0.0
        self.timesRun = 0
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setRealTimeSimulation(realWorld)
        self.resetState = pybullet.saveState()
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def computeState(self):
        bodyState = pybullet.getLinkState(self.dog, 0)
        bodypos = bodyState[0]
        bodyquat = bodyState[1]
        self.state = {"JointPosition": [pybullet.getJointState(self.dog, i)[0] for i in range(self.numJoints)],
                      "JointVelocity":  [pybullet.getJointState(self.dog, i)[1] for i in range(self.numJoints)],
                      "bodyPos": bodypos,
                      "bodyquat": bodyquat}
        self.stateToReturn = np.array(
            [self.state['JointPosition'][0:4], self.state['JointPosition'][5:9], self.state['bodyquat']]).flatten()

    def step(self, movement, max_steps):
        pybullet.stepSimulation()
        self.currentSimTime += self.timeStep
        self.timesRun += 1
        bodyx = self.state["bodyPos"][0]

        action = self.stateToReturn[0:8]
        for i in range(8):
            action[i] += movement[i]*0.01

        # pybullet.setJointMotorControl2(self.dog, 4, pybullet.POSITION_CONTROL, targetPosition=0, force=500)
        # pybullet.setJointMotorControl2(self.dog, 9, pybullet.POSITION_CONTROL, targetPosition=0, force=500)

        
        for i in range(4):
            pybullet.setJointMotorControl2(self.dog, i, pybullet.POSITION_CONTROL, targetPosition=action[i], force=500)
        
        for i in range(4):
            pybullet.setJointMotorControl2(self.dog, i+5, pybullet.POSITION_CONTROL, targetPosition=action[i+4], force=500)

        self.computeState()
        
        pos, rot = pybullet.getBasePositionAndOrientation(self.dog)
        rotmat = pybullet.getMatrixFromQuaternion(rot)
        upv = np.array([rotmat[2], rotmat[5], rotmat[8]])
        hasFallenOrient = False
        # we compute the dot product of 0 0 1 with upv
        # the angle between the global z axis and the z axis of the base is the arccos of this dotproduct
        # 0.95 ~ cos( 18.5° )
        if(upv[2] < 0.9):
            hasFallenOrient = True
        hasFallen = self.state["bodyPos"][2] < 0.8
        done = self.timesRun > max_steps or hasFallen or hasFallenOrient or np.isnan(
            bodyx)

        #REWARD FUNCTION
        Wvel = 2.0
        We = 0.05*0
        delX = np.nan_to_num(pos[0]-self.previousPos[0])
        delE = 0

        for i in range(4):
            delE += pybullet.getJointState(self.dog, i)[3]        
        for i in range(4):
            delE += pybullet.getJointState(self.dog, i+5)[3]

        reward = Wvel*np.sign(delX)*max(abs(delX), 0.1)-We*delE

        if hasFallen or hasFallenOrient:
            reward = reward - 20.0

        self.previousPos = pos

        return self.stateToReturn, reward, done, {}

    def reset(self):
        self.currentSimTime = 0.0
        self.timesRun = 0
        
        pybullet.restoreState(self.resetState)

        for i in range(self.numJoints):
            pybullet.resetJointState(self.dog, i, 0, 0)
        
        for i in range(4):
            pybullet.setJointMotorControl2(self.dog, i, pybullet.VELOCITY_CONTROL, targetVelocity=0, force=500000000)
            pybullet.setJointMotorControl2(self.dog, i+5, pybullet.VELOCITY_CONTROL, targetVelocity=0, force=500000000)
            pybullet.setJointMotorControl2(self.dog, i, pybullet.POSITION_CONTROL, targetPosition=0, force=500000000)
            pybullet.setJointMotorControl2(self.dog, i+5, pybullet.POSITION_CONTROL, targetPosition=0, force=500000000)

        pos, rot = pybullet.getBasePositionAndOrientation(self.dog)
        self.previousPos = pos
        self.state = {}
        self.computeState()
        

        return self.stateToReturn

