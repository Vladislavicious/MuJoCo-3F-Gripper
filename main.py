import os
import sys
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'manipulator_grasp'))

import numpy as np
import spatialmath as sm
from scipy.linalg import orthogonal_procrustes

from manipulator_grasp.arm.motion_planning import *
from manipulator_grasp.env.ur5_grasp_env import UR5GraspEnv

from utils_wrapper.motion_plan import *
from utils_wrapper.grasp_net import *


class MyRobot:
    def __init__(self, env:UR5GraspEnv): 
        self.env = env
        self.robot = env.robot

        
        self.home_joints = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])

        self.action = np.zeros(6+1)
        self.joints = self.robot.get_joint()
        self.tcp_pose = self.robot.get_cartesian()
    def move_p(self, target_pose, run_time=2):
        self.action = move_p(self.robot, self.env, target_pose, run_time, self.action)

    def move_j(self, target_joints, run_time=2):
        self.action = move_j(self.robot, self.env, target_joints, run_time, self.action)

    def close(self):
        # 夹爪闭合
        for i in range(1500):
            self.action[-1] += 0.2
            self.action[-1] = np.min([self.action[-1], 255])
            env.step(self.action)
    def open(self):
        # 夹爪打开
        for i in range(1500):
            self.action[-1] -= 0.2
            self.action[-1] = np.max([self.action[-1], 0])
            env.step(self.action)


if __name__ == '__main__':
    env = UR5GraspEnv()
    env.reset()
    for i in range(1000):
        env.step()

    robot = MyRobot(env)
    
    # 红色小方块的抓取位姿
    R = np.array([
        [ 0.6018,    0.0782,   -0.7948],
        [ 0.0159,   -0.9962,   -0.08597],
        [-0.7985,    0.0391,   -0.6007]
    ])
    R_normalized, _ = orthogonal_procrustes(R, np.eye(3))  # 正交化 R
    t = np.array([1.382, 0.1989, 0.7914])

    # 构造 SE3
    T = sm.SE3(t) * sm.SE3(sm.SO3(R_normalized))
    
    
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    # 第一阶段
    robot.move_j(robot.home_joints)
    # 第二阶段
    target_pose = T * sm.SE3(-0.1, 0.0, 0.0)
    robot.move_p(target_pose)
    # 第三阶段
    T3 = T
    robot.move_p(T3)
    # 夹爪闭合
    robot.close()
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    T4 = sm.SE3.Trans(0.0, 0.0, 0.1) * T3
    robot.move_p(T4)

    T5 = sm.SE3.Trans(1.4, 0.2, T4.t[2]) * sm.SE3(sm.SO3(T4.R))
    robot.move_p(T5)

    T6 = sm.SE3.Trans(0.2, 0.2, T5.t[2]) * sm.SE3(sm.SO3.Rz(-np.pi / 2) * sm.SO3(T5.R))
    robot.move_p(T6)

    
    T7 = sm.SE3.Trans(0.0, 0.0, -0.1) * T6
    robot.move_p(T7)

    robot.open()

    
    T8 = sm.SE3.Trans(0.0, 0.0, 0.2) * T7
    robot.move_p(T8)

    robot.move_j(robot.home_joints)


    for i in range(2000):
        env.step()

    env.close()
