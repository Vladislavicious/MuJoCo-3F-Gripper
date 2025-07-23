import os
import sys
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'manipulator_grasp'))

import numpy as np
import spatialmath as sm

from manipulator_grasp.arm.motion_planning import *
from manipulator_grasp.env.ur5e_3f_grasp_env import UR5GraspEnv

from utils_wrapper.motion_plan import *

class MyRobot:
    def __init__(self, env:UR5GraspEnv): 
        self.env = env
        self.robot = env.robot

        
        self.home_joints = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])

        self.action = np.zeros(6+4)
        self.joints = self.robot.get_joint()
        self.tcp_pose = self.robot.get_cartesian()
    def move_p(self, target_pose, run_time=2):
        self.action = move_p(self.robot, self.env, target_pose, run_time, self.action)

    def move_j(self, target_joints, run_time=2):
        self.action = move_j(self.robot, self.env, target_joints, run_time, self.action)

    def close(self):
        # 夹爪闭合
        self.action[-4:] = [0, 0, 0, -0.297]
        for i in range(1500):
            self.action[-4] += 0.2
            self.action[-3] += 0.2
            self.action[-2] += 0.2

            send_action = self.action.copy()
            send_action[-4] = np.min([self.action[-4], 255]) / 255 * 0.53
            send_action[-3] = np.min([self.action[-3], 255]) / 255 * 0.53
            send_action[-2] = np.min([self.action[-2], 255]) / 255 * 0.53

            env.step(send_action)
    def open(self):
        # 夹爪打开
        for i in range(1500):
            self.action[-4] -= 0.2
            self.action[-3] -= 0.2
            self.action[-2] -= 0.2

            send_action = self.action.copy()
            send_action[-4] = np.max([self.action[-4], 0]) / 255 * 0.53
            send_action[-3] = np.max([self.action[-3], 0]) / 255 * 0.53
            send_action[-2] = np.max([self.action[-2], 0]) / 255 * 0.53

            env.step(send_action)

if __name__ == '__main__':
    env = UR5GraspEnv()
    env.reset()
    
    for i in range(1000):
        env.step()
    

    robot = MyRobot(env)

    # 第一阶段
    robot.move_j(robot.home_joints)
    # 第二阶段
    target_pose = robot.robot.get_cartesian() * sm.SE3(0.25, -0.27, -0.16) # X 是-z y 是-y z是-x
    print(target_pose)
    robot.move_p(target_pose)
    # # 第三阶段
    # T3 = T_wo
    # robot.move_p(T3)
    # 夹爪闭合
    robot.close()

    target_pose = robot.robot.get_cartesian() * sm.SE3(-0.2, 0, 0 ) # X 是-z y 是-y z是-x
    print(target_pose)
    robot.move_p(target_pose)

    # robot.open()


    for i in range(200000):
        env.step()

    env.close()
