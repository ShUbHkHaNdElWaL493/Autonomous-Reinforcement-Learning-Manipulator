#!/usr/bin/env python3

#   CS22B1090
#   Shubh Khandelwal

from collections import deque, namedtuple
from env_node import ManipulatorEnv
import numpy as np
import random
import rclpy
import time
import torch
import torch.nn as nn
import torch.optim as optim

class DeepQNetwork(nn.Module):

    def __init__(self, observation_size, action_size):
        super(DeepQNetwork, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(observation_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_size)
        )
    
    def forward(self, x):
        return self.layers(x)

class ReplayBuffer:

    def __init__(self, buffer_size, batch_size):
        self.memory = deque(maxlen = buffer_size)
        self.batch_size = batch_size
        self.experience = namedtuple("Experience", field_names = ["state", "action", "reward", "next_state", "done"])

    def __len__(self):
        return len(self.memory)

    def add(self, state, action, reward, next_state, done):
        self.memory.append(self.experience(state, action, reward, next_state, done))

    def sample(self):
        experiences = random.sample(self.memory, k = self.batch_size)
        states = torch.stack([torch.from_numpy(e.state) for e in experiences if e is not None]).float()
        actions = torch.stack([torch.from_numpy(np.array([e.action], dtype = np.int32)) for e in experiences if e is not None]).long()
        rewards = torch.stack([torch.from_numpy(np.array([e.reward], dtype = np.float32)) for e in experiences if e is not None]).float()
        next_states = torch.stack([torch.from_numpy(e.next_state) for e in experiences if e is not None]).float()
        dones = torch.stack([torch.from_numpy(np.array([float(e.done)], dtype = np.float32)) for e in experiences if e is not None]).float()
        return (states, actions, rewards, next_states, dones)

class DQNAgent:

    def __init__(self, state_size, action_size, buffer_size = int(1e5), batch_size = 32, gamma = 0.99, tau = 1e-3, lr = 5e-4, update_after_steps = 4):

        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = batch_size
        self.gamma = gamma
        self.tau = tau
        self.lr = lr
        self.update_after_steps = update_after_steps

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.memory = ReplayBuffer(buffer_size, batch_size)
        self.network_local = DeepQNetwork(state_size, action_size).to(self.device)
        self.network_target = DeepQNetwork(state_size, action_size).to(self.device)
        self.optimizer = optim.Adam(self.network_local.parameters(), lr = self.lr)
        self.steps = 0

    def act(self, state, epsilon):
        state = torch.from_numpy(state).float()
        state = state.unsqueeze(0).to(self.device)
        self.network_local.eval()
        with torch.no_grad():
            action_values = self.network_local(state)
        self.network_local.train()
        if random.random() > epsilon:
            return torch.argmax(action_values).item()
        else:
            return random.randrange(self.action_size)
        
    def soft_update(self):
        for target_parameters, local_parameters in zip(self.network_target.parameters(), self.network_local.parameters()):
            target_parameters.data.copy_(self.tau * local_parameters.data + (1.0 - self.tau) * target_parameters.data)
    
    def learn(self, experiences):

        states, actions, rewards, next_states, dones = experiences
        states = states.to(self.device)
        actions = actions.to(self.device)
        rewards = rewards.to(self.device)
        next_states = next_states.to(self.device)
        dones = dones.to(self.device)

        targets_next = self.network_target(next_states).detach().max(1)[0].unsqueeze(1)
        target_values = rewards + (self.gamma * targets_next * (1 - dones))
        predicted_values = self.network_local(states).gather(1, actions)

        loss = nn.functional.mse_loss(predicted_values, target_values)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        self.soft_update()

    def step(self, state, action, reward, next_state, done):
        self.memory.add(state, action, reward, next_state, done)
        self.steps = (self.steps + 1) % self.update_after_steps
        if self.steps == 0:
            if len(self.memory) > self.batch_size:
                experiences = self.memory.sample()
                self.learn(experiences)

def main(args = None):

    rclpy.init(args = args)
    manipulator_env = ManipulatorEnv()
    model = DQNAgent(int(np.prod(manipulator_env.observation_space.shape)), manipulator_env.action_space.n)

    try:

        episodes = 10
        for episode in range(episodes):

            manipulator_env.get_logger().info(f"\nEpisode: {episode + 1}/{episodes}")
            state, info = manipulator_env.reset()
            manipulator_env.get_logger().info(f"Episode: {episode + 1}| Initial Gripper Position: {state}")
            manipulator_env.get_logger().info(f"Episode: {episode + 1}| Initial Goal Position: {manipulator_env.goal}")

            episode_reward = 0
            done = False
            while not done:
                action = model.act(state, epsilon = 1)
                next_state, reward, terminated, truncated, info = manipulator_env.step(action)
                episode_reward += reward
                done = terminated or truncated
                model.step(state, action, reward, next_state, done)
                state = next_state

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