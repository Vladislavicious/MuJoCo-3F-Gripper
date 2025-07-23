import os
import sys
import numpy as np
import spatialmath as sm


from manipulator_grasp.arm.motion_planning import *

def execute_trajectory(robot, env, planners, time_array, action=np.zeros(7)):
        """
        执行多个轨迹规划器定义的轨迹
        
        参数:
            robot: 机器人对象
            env: 环境对象
            planners: 包含多个 TrajectoryPlanner 实例的列表
            time_array: 每个阶段的时间数组，如 [0, time0, time1, time2]
            action: 动作数组，用于控制机器人（包含夹爪）
        """
        total_time = np.sum(time_array)
        time_step_num = round(total_time / 0.002) + 1
        times = np.linspace(0, total_time, time_step_num)
        time_cumsum = np.cumsum(time_array)
        
        # # action 前6个是关节，最后一个是夹爪的开闭
        for timei in times:
            for j in range(len(time_cumsum)):
                if timei < time_cumsum[j]:
                    planner = planners[j - 1]
                    planner_interpolate = planner.interpolate(timei - time_cumsum[j - 1])
                    if isinstance(planner_interpolate, np.ndarray):
                        joint = planner_interpolate
                        robot.move_joint(joint)
                    else:
                        robot.move_cartesian(planner_interpolate)
                        joint = robot.get_joint()
                    action[:6] = joint
                    env.step(action)
                    break
        return action
def move_p(robot, env, target_pose, run_time, action=np.zeros(7)):
    time = run_time
    T1 = robot.get_cartesian()
    T2 = target_pose # 必须和robot.get_cartesian()类型一样

    position_parameter = LinePositionParameter(T1.t, T2.t)
    attitude_parameter = OneAttitudeParameter(sm.SO3(T1.R), sm.SO3(T2.R))
    cartesian_parameter = CartesianParameter(position_parameter, attitude_parameter)

    velocity_parameter = QuinticVelocityParameter(time)

    trajectory_parameter = TrajectoryParameter(cartesian_parameter, velocity_parameter)
    planner = TrajectoryPlanner(trajectory_parameter)

    action = execute_trajectory(robot, env, [planner], [0, time], action) 
    return action

def move_j(robot, env, target_joint, run_time, action=np.zeros(7)):
    time = run_time
    q0 = robot.get_joint()
    q1 = target_joint # np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])

    parameter = JointParameter(q0, q1)
    velocity_parameter = QuinticVelocityParameter(time)
    trajectory_parameter = TrajectoryParameter(parameter, velocity_parameter)
    planner = TrajectoryPlanner(trajectory_parameter)
    return execute_trajectory(robot, env, [planner], [0, time], action)