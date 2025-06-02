#!/usr/bin/env python3

#   CS22B1090
#   Shubh Khandelwal

import gymnasium as gym
from manipulator.srv import Gripper
import numpy as np
import rclpy
from rclpy.node import Node
import time

class ManipulatorEnv(gym.Env, Node):

    def __init__(self):

        Node.__init__(self, "manipulator_node")
        gym.Env.__init__(self)

        self.lower_limits = np.array([-3.142, 0, 0, 0], dtype = np.float32)
        self.upper_limits = np.array([3.142, 1.571, 1.571, 1.571], dtype = np.float32)

        self.observation_space = gym.spaces.Box(low = -np.inf, high = np.inf, shape = (3, ), dtype = np.float32)
        self.action_space = gym.spaces.Discrete(81)

        self.joints = np.zeros(4, dtype = np.float32)
        self.gripper = np.zeros(3, dtype = np.float32)
        self.goal = np.zeros(3, dtype = np.float32)
        self.initial_d = 0
        self.steps = 0

        self.gripper_client = self.create_client(Gripper, '/get_gripper')
        self.request = Gripper.Request()
    
    def _get_gripper(self):
        self.request.joints = self.joints
        future = self.gripper_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        while future.result() is None or not future.result().success:
            future = self.gripper_client.call_async(self.request)
            rclpy.spin_until_future_complete(self, future)
        self.gripper = future.result().gripper
        
    def reset(self, seed = None, options = None):
        super().reset(seed = seed)
        self.joints = np.random.uniform(low = self.lower_limits, high = self.upper_limits, size = (4, )).astype(np.float32)
        self.goal = np.random.uniform(low = [-0.45, -0.45, 0.4], high = [0.45, 0.45, 0.85], size = (3, )).astype(np.float32)
        self.steps = 0
        self.initial_d = np.sqrt(np.sum((self.gripper - self.goal) ** 2))
        self._get_gripper()
        return self.gripper, {}
    
    def step(self, action):

        step_size = 0.01
        action1 = action % 3
        action = (action - action1) / 3
        action2 = action % 3
        action = (action - action2) / 3
        action3 = action % 3
        action = (action - action3) / 3
        action4 = action % 3
        delta = np.array([action1, action2, action3, action4], dtype = np.float32) - 1
        self.joints += delta * step_size
        self.joints = np.clip(self.joints, self.lower_limits, self.upper_limits)
        self.steps += 1

        self._get_gripper()

        terminated = False
        truncated = False
        reward = 0
        for i in range(4):
            reward += (i + 1) * abs(delta[i])
        distance = np.sqrt(np.sum((self.gripper - self.goal) ** 2))
        reward -= distance * 100
        reward += (distance - self.initial_d) * 10
        if distance <= 0.1:
            reward = 100000
            terminated = True
        elif self.steps >= 10000:
            truncated = True
        self.initial_d = distance
        
        return self.gripper, reward, terminated, truncated, {}

def main(args = None):

    rclpy.init(args = args)
    manipulator_env = ManipulatorEnv()

    try:

        episodes = 5

        for episode in range(episodes):

            manipulator_env.get_logger().info(f"\nEpisode: {episode + 1}/{episodes}")
            state, info = manipulator_env.reset()
            manipulator_env.get_logger().info(f"Episode: {episode + 1} | Initial Gripper Position: {state}")
            manipulator_env.get_logger().info(f"Episode: {episode + 1} | Initial Goal Position: {manipulator_env.goal}")

            episode_reward = 0
            done = False
            while not done:
                action = manipulator_env.action_space.sample()
                state, reward, terminated, truncated, info = manipulator_env.step(action)
                episode_reward += reward
                done = terminated or truncated

            manipulator_env.get_logger().info(f"Episode: {episode + 1} | Steps: {manipulator_env.steps} | Reward: {episode_reward}")
            if terminated:
                manipulator_env.get_logger().info("Episode terminated.")
            elif truncated:
                manipulator_env.get_logger().info("Episode truncated.")
            time.sleep(1)

    except Exception as e:
        manipulator_env.get_logger().error(f"Error: {e}", once=True)
    finally:
        manipulator_env.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()