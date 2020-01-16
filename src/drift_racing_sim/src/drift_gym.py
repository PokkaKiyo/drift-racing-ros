#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

import gym
import numpy as np
import subprocess
import time
import os

from my_utils import get_figure_eight_coordinates

class GazeboEnv(gym.Env):
    def __init__(self):
        rospy.init_node('gazebo_drift_car_gym')

        self._cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.ackermann_cmd_msg = AckermannDrive()
        self.ackermann_cmd_msg.speed = 1.0

        self.pause_physics()

    def drive_car(self, action):
        if action == 1:
            self.ackermann_cmd_msg.steering_angle = 0.8
        elif action == 2:
            self.ackermann_cmd_msg.steering_angle = -0.8
        else:
            self.ackermann_cmd_msg.steering_angle = 0

        self._cmd_vel_pub.publish(self.ackermann_cmd_msg)

    def step(self, action):
        self.unpause_physics()

        self.drive_car(action)

        state = self.get_state()
        reward = self.get_reward()
        done = self.get_isdone()
        info = None

        self.pause_physics()

        return state, reward, done, info


    def get_state(self):
        model_states = self.get_model_states()
        state = self.get_car_state(model_states)
        return state

    def get_reward(self):
        drift_metric_score = self.get_drift_metric_score()
        path_tracking_score = self.get_path_tracking_score()

        alpha = 0.5
        reward = (alpha * drift_metric_score) + ((1.0 - alpha) * path_tracking_score)

        return reward
    
    def get_isdone(self):
        return False # stub
    
    def get_drift_metric_score(self):
        return 0 # stub
    
    def get_path_tracking_score(self):
        return 0 # stub

    def get_model_states(self):
        num_tries = 0
        model_states = None
        while model_states is None:
            try:
                num_tries += 1
                model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10)
            except Exception as e:
                pass
        
        if num_tries > 1:
            print("get_model_state num_tries:", num_tries)
        
        return model_states

    def get_car_state(self, model_states):
        '''
        The car state consists of:
        1. x coordinate (world)
        2. y coordinate (world)
        3. velocity x-axis (baselink)
        4. velocity y-axis (baselink)
        5. yaw (baselink)
        '''
        car_state = []

        car_pose = model_states.pose[0]
        car_position = car_pose.position

        car_state.append(car_position.x)
        car_state.append(car_position.y)

        # do transform to base link frame, then add the other 3 state components

        return car_state

    def reset_env(self):
        print('resetting env')
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            reset_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState()
            model_state.model_name = "ackermann_vehicle"
            model_state.pose.position.x = 0
            model_state.pose.position.y = 0
            model_state.pose.position.z = 0
            reset_pose(model_state)
        except (rospy.ServiceException) as e:
            print("/gazebo/set_model_state service call failed")
        
        model_states = self.get_model_states()
        state = self.get_car_state(model_states)
        print('done')
        return state

    def pause_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
         
    def unpause_physics(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

    def _close(self):
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count('gzclient')
        gzserver_count = tmp.count('gzserver')
        roslaunch_count = tmp.count('roslaunch')

        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        if roslaunch_count > 0:
            os.system("killall -9 roslaunch")

        if (gzclient_count or gzserver_count or roslaunch_count):
            os.wait()
