#!/usr/bin/env python

import rospy
import tf
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

import gym
import numpy as np
import subprocess
import time
import os
import math

from my_utils import get_figure_eight_coordinates, calculateL2Dist

class GazeboEnv(gym.Env):
    def __init__(self):
        rospy.init_node('gazebo_drift_car_gym')

        self._cmd_vel_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.ackermann_cmd_msg = AckermannDrive()
        self.ackermann_cmd_msg.speed = 1.0

        self.path_coordinates = get_figure_eight_coordinates()
        self.waypoint_idx = 0

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
        reward = self.get_reward(state)
        done = self.get_isdone()
        info = None

        self.pause_physics()

        return state, reward, done, info


    def get_state(self):
        model_states = self.get_model_states()
        state = self.get_car_state(model_states)
        return state

    def get_reward(self, state):
        drift_metric_score = self.get_drift_metric_score(state)
        path_tracking_score = self.get_path_tracking_score(state)

        alpha = 0.5
        reward = (alpha * drift_metric_score) + ((1.0 - alpha) * path_tracking_score) - 1

        return reward
    
    def get_isdone(self):
        if self.dist_to_waypoint > 5.0:
            print("Done:", self.dist_to_waypoint)
            return True
        return False
    
    # For now, using the one from https://github.com/kanakkabara/Autonomous-Drifting
    def get_drift_metric_score(self, state):
		desiredForwardVel = 0.5		
		desiredSideVel = 2  
        desiredAngularVel = -3.5

		carForwardVel = state[3]
		carSideVel = state[4]
        carAngularVel = state[5]

        sigma = 5
        deviationMagnitude = (carSideVel - desiredSideVel)**2 + \
                        (carForwardVel - desiredForwardVel)**2 + \
                        (carAngularVel - desiredAngularVel)**2
        
        return 1 - math.exp(-deviationMagnitude/(2 * sigma**2))

    def get_path_tracking_score(self):
        current_x = state[0]
        current_y = state[1]
        waypoint_x, waypoint_y = self.path_coordinates[self.waypoint_idx]
        dist_to_waypoint = calculateL2Dist(current_x, waypoint_x, current_y, waypoint_y)
        self.dist_to_waypoint = dist_to_waypoint
    
        if (np.abs(current_x - waypoint_x) + np.abs(current_y - waypoint_y)) < 1e-3:
            self.waypoint_idx = (self.waypoint_idx + 1) % len(self.path_coordinates)
        
        return -1 * dist_to_waypoint

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
        car_position = model_states.pose[1].position
        car_orientation = model_states.pose[1].orientation

        t = tf.TransformerROS(True, rospy.Duration(10.0))
        m = TransformStamped()
        m.header.frame_id = 'world'
        m.child_frame_id = 'base_link'
        m.transform.translation.x = car_position.x
        m.transform.translation.y = car_position.y
        m.transform.translation.z = car_position.z
        m.transform.rotation.x = car_orientation.x
        m.transform.rotation.y = car_orientation.y
        m.transform.rotation.z = car_orientation.z
        m.transform.rotation.w = car_orientation.w
        t.setTransform(m)

        vel_vector = Vector3Stamped()
        vel_vector.vector.x = model_states.twist[1].linear.x
        vel_vector.vector.y = model_states.twist[1].linear.y
        vel_vector.vector.z = model_states.twist[1].linear.z
        vel_vector.header.frame_id = "world"
        vel_vector_baselink = t.transformVector3("base_link", vel_vector)

        car_yaw = model_states.twist[1].angular.z

        car_state = []
        car_state.append(car_position.x)
        car_state.append(car_position.y)
        car_state.append(vel_vector_baselink.vector.x)
        car_state.append(vel_vector_baselink.vector.y)
        car_state.append(car_yaw)

        return car_state

    def reset_env(self):
        print('resetting env...')
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
        print('...done')
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
