#!/usr/bin/env python

from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from collections import deque
import numpy as np
import random

class DQNAgent:
    def __init__(self,
                n_features=5,
                n_actions=3,
                learning_rate=0.001,
                discount_factor=0.99,
                epsilon_start=1.0,
                epsilon_decay=0.95,
                epsilon_min=0.01,
                memory_size=10000,
                batch_size=32,
                train_start=10,
                load_models=False,
                model_path=None
                ):

        self.n_features = n_features
        self.n_actions = n_actions
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor

        self.epsilon = epsilon_start
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min

        self.memory = deque(maxlen=memory_size)
        self.batch_size = batch_size
        self.train_start = train_start

        if load_models == False:
            self.model = self.build_model()
            self.target_model = self.build_model()
            self.update_target_model()
        else:
            self.model = load_model(model_path)
            self.target_model = load_model(model_path)

    def build_model(self):
        model = Sequential()
        model.add(Dense(64, input_dim=self.n_features, activation='relu',
                        kernel_initializer='he_uniform'))
        model.add(Dense(32, activation='relu',
                        kernel_initializer='he_uniform'))
        model.add(Dense(self.n_actions, activation='linear',
                        kernel_initializer='he_uniform'))
        model.summary()
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))
        return model

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            return random.randrange(self.n_actions)
        else:
            q_value = self.model.predict(state)
            return np.argmax(q_value[0])
    
    def memorise(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        self.epsilon = max(self.epsilon_min, (self.epsilon * self.epsilon_decay))


    def learn(self):
        if len(self.memory) < self.train_start:
            return
        
        minibatch = random.sample(self.memory, self.batch_size)

        current_states = np.zeros((self.batch_size, self.n_features))
        future_states = np.zeros((self.batch_size, self.n_features))
        action_list, reward_list, done_list = [], [], []

        for i in range(self.batch_size):
            current_states[i] = minibatch[i][0]
            action_list.append(minibatch[i][1])
            reward_list.append(minibatch[i][2])
            future_states[i] = minibatch[i][3]
            done_list.append(minibatch[i][4])
        
        target = self.model.predict(current_states)
        target_val = self.target_model.predict(future_states)

        for i in range(self.batch_size):
            if done_list[i]:
                target[i][action_list[i]] = reward_list[i]
            else:
                target[i][action_list[i]] = reward_list[i] + self.discount_factor * (
                    np.amax(target_val[i]))
        
        self.model.fit(current_states, target, batch_size=self.batch_size,
                epochs=1, verbose=0)


    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())
    
    def save_model(self, model_path=None):
        if model_path is None:
            print("Not saving as model_path is not specified")
        else:
            self.model.save(filepath=model_path)
