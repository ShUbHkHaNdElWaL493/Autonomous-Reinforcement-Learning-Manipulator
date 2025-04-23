#!/usr/bin/env python3

#   CS22B1090
#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import gymnasium as gym
import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from tf2_ros import Buffer, TransformListener
import threading
import time

class ManipulatorEnv(gym.Env):

    def __init__(self):

        super().__init__()

        self.lower_limits = np.array([-3.142, 0, 0, 0], dtype = np.float32)
        self.upper_limits = np.array([3.142, 1.571, 1.571, 1.571], dtype = np.float32)

        self.observation_space = gym.spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.inf], dtype=np.float32),
            high=np.array([np.inf, np.inf, np.inf], dtype=np.float32),
            dtype=np.float32
        )
        self.action_space = gym.spaces.MultiDiscrete([3] * 4)

        self.joint_states = np.zeros(4, dtype = np.float32)
        self.gripper_state = np.zeros(3, dtype = np.float32)
        self.goal_state = np.zeros(3, dtype = np.float32)
        self.initial_d = 0
        self.steps = 0
        
    def reset(self, seed = None, options = None):
        super().reset(seed=seed)
        self.joint_states = np.zeros(4, dtype = np.float32)
        self.goal_state = np.random.uniform(low = -0.4, high = 0.4, size = (3, )).astype(np.float32)
        self.steps = 0
        time.sleep(0.001)
        self.initial_d = np.sqrt(np.sum((self.gripper_state - self.goal_state) ** 2))
        return self.gripper_state, {}
    
    def step(self, action):

        delta = np.array(action) - 1
        step_size = 0.01

        self.joint_states += delta * step_size
        self.joint_states = np.clip(self.joint_states, self.lower_limits, self.upper_limits)
        self.steps += 1

        time.sleep(0.001)

        reward = 0
        terminated = False
        truncated = False
        distance = np.sqrt(np.sum((self.gripper_state - self.goal_state) ** 2))
        reward = self.initial_d - distance
        if distance <= 0.01:
            reward = 1000
            terminated = True
        elif self.steps > 1000:
            truncated = True
        self.initial_d = distance
        
        return self.gripper_state, reward, terminated, truncated, {}

class JointStateNode(Node):

    def __init__(self, manipulator_env):
        super().__init__('joint_state_node')
        self.manipulator_env = manipulator_env
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.001, self.publish_joint_states)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ["joint_01", "joint_23", "joint_45", "joint_67"]
        self.joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0]

    def publish_joint_states(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = list(self.manipulator_env.joint_states)
        self.publisher.publish(self.joint_state_msg)

class GripperPositionNode(Node):

    def __init__(self, manipulator_env):
        super().__init__('gripper_position_node')
        self.manipulator_env = manipulator_env
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.001, self.get_gripper_position)

    def get_gripper_position(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'world',
                'link_8',
                rclpy.time.Time()
            )
            self.manipulator_env.gripper_state = np.array(
                [
                    tf_msg.transform.translation.x,
                    tf_msg.transform.translation.y,
                    tf_msg.transform.translation.z
                ], dtype=np.float32
            )
        except Exception as e:
            self.get_logger().warn(f"Could not find transform: {e}")

class GoalStateNode(Node):

    def __init__(self, manipulator_env):
        super().__init__('goal_state_node')
        self.manipulator_env = manipulator_env
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.goal_state_callback, 10)

    def goal_state_callback(self, msg):
        self.manipulator_env.goal_state = np.array([msg.point.x, msg.point.y, msg.point.z], dtype = np.float32)
        self.get_logger().info(f"Goal state: [{msg.point.x}, {msg.point.y}, {msg.point.z}]")

def main(args=None):

    rclpy.init(args=args)
    manipulator_env = ManipulatorEnv()
    monitored_env = Monitor(manipulator_env)
    manipulator_vector_env = DummyVecEnv([lambda: monitored_env])

    package_share_directory = get_package_share_directory("manipulator_controller")
    model_path = os.path.abspath(os.path.join(package_share_directory, "models", "manipulator.zip"))
    model = PPO.load(model_path, env = manipulator_vector_env)

    joint_state_node = JointStateNode(manipulator_env)
    time.sleep(1)
    gripper_position_node = GripperPositionNode(manipulator_env)
    time.sleep(1)
    goal_state_node = GoalStateNode(manipulator_env)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joint_state_node)
    executor.add_node(gripper_position_node)
    executor.add_node(goal_state_node)

    def ros_spin():
        executor.spin()

    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    state = manipulator_vector_env.reset()
    while rclpy.ok():
        done = False
        while not done:
            action, _ = model.predict(state, deterministic = True)
            state, _, done, _, _ = manipulator_vector_env.step(action)

    executor.shutdown()
    joint_state_node.destroy_node()
    gripper_position_node.destroy_node()
    goal_state_node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == "__main__":
    main()