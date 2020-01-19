#!/usr/bin/env python

from dqn_agent import DQNAgent
from drift_gym import GazeboEnv
import numpy as np

# HYPER-PARAMETERS
EPISODES = 2

LEARNING_RATE = 0.001
DISCOUNT_FACTOR = 0.99
EPSILON_START = 1.0
EPSILON_DECAY = 0.999
EPSILON_MIN = 0.01
MEMORY_SIZE = 50000
BATCH_SIZE = 128
TRAIN_START = 200


if __name__ == '__main__':
    env = GazeboEnv()

    agent = DQNAgent(
                    n_features=5,
                    n_actions=3,
                    learning_rate=LEARNING_RATE,
                    discount_factor=DISCOUNT_FACTOR,
                    epsilon_start=EPSILON_START,
                    epsilon_decay=EPSILON_DECAY,
                    epsilon_min=EPSILON_MIN,
                    memory_size=MEMORY_SIZE,
                    batch_size=BATCH_SIZE,
                    train_start=TRAIN_START,
                    load_models=False,
                    model_path=None
                    )

    for episode in range(EPISODES):
        state = env.reset()
        state = np.reshape(state, [1, 5])
        done = False
        num_steps = 0
        episode_reward = 0
        while not done:
            action = agent.choose_action(state)
            next_state, reward, done, _ = env.step(action)
            next_state = np.reshape(next_state, [1, 5])

            agent.memorise(state, action, reward, next_state, done)
            agent.learn()

            state = next_state
            episode_reward += reward

            num_steps += 1
            if num_steps >= 200:
                done = True

            if done:
                print("Done with episode", episode)
                agent.update_target_model()

