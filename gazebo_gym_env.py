#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

import numpy as np
import subprocess
import time
import os

import gym

class GazeboEnv(gym.Env):
    def __init__(self):
        rospy.init_node('gazebo_drift_car_gym')

        # self._cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1) # check the topic name
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.pause_physics()

    def drive_car(self, throttle, steering):
        pass

    def step(self, action):
        self.unpause_physics()
        self.drive_car(action[0], action[1])
        self.pause_physics()

        state = self.get_state()
        reward = self.get_reward()
        done = self.get_isdone()

        info = None
        return state, reward, done, info


    def get_state(self):
        pass

    def get_reward(self):
        drift_metric_score = self.get_drift_metric_score()
        path_tracking_score = self.get_path_tracking_score()

        alpha = 0.5
        reward = (alpha * drift_metric_score) + ((1.0 - alpha) * path_tracking_score)

        return reward
    
    def get_isdone(self):
        pass
    
    def get_drift_metric_score(self):
        pass
    
    def get_path_tracking_score(self):
        pass

    def reset_episode(self):
        pass

    def pause_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except (rospy.ServiceException) as e:
                print("/gazebo/pause_physics service call failed")
         
    def unpause_physics(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
            try:
                self.pause()
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

if __name__ == "__main__":
    env = GazeboEnv()
    env.reset_episode()

