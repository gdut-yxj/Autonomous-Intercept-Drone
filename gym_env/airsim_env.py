import math
import time
import airsim
import cv2
import torch
from PIL import Image
import numpy as np
import gym
from gym import spaces
from configparser import NoOptionError
# from .SimpleMultirotor import MultirotorDynamicsSimple
from .SimpleMultirotorVel import MultirotorDynamicsVel

from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal


class AirsimEnv(gym.Env, QtCore.QThread):
    # pyqt signal for visualization
    action_signal = pyqtSignal(int, np.ndarray)
    state_signal = pyqtSignal(int, np.ndarray)
    attitude_signal = pyqtSignal(int, np.ndarray, np.ndarray)
    reward_signal = pyqtSignal(int, float, float)

    def __init__(self) -> None:
        super().__init__()
        np.set_printoptions(formatter={'float': '{: 4.2f}'.format}, suppress=True)
        torch.set_printoptions(profile="short", sci_mode=False, linewidth=1000)
        print("init airsim_gym_env")
        self.model = None
        self.data_path = None
        self.cfg = None
        self.env_name = None
        self.perception_type = None
        self.dynamic_model = None
        self.goal_distance = 0

    def set_config(self, cfg):
        self.cfg = cfg
        self.env_name = cfg.get('options', 'env_name')
        self.perception_type = cfg.get('options', 'perception')
        print('Environment: ', self.env_name, "Perception: ", self.perception_type)
        # self.dynamic_model = MultirotorDynamicsSimple(cfg)
        self.dynamic_model = MultirotorDynamicsVel(cfg)

        if self.env_name == 'Forest':
            start_position = [0, 0, 1]
            # goal_position = [20, 27, 1]
            self.goal_distance = 62
            # self.dynamic_model.set_start(start_position, random_angle=math.pi / 2)
            # self.dynamic_model._set_goal_pose_single(goal_position)
            self.dynamic_model.set_start(start_position, random_angle=0)
            self.dynamic_model.set_goal(distance=self.goal_distance, random_angle=0)
            # self.dynamic_model.set_goal(distance=goal_distance, random_angle=math.pi * 2)
            self.work_space_x = [start_position[0] - 1, start_position[0] + 64]
            # self.work_space_y = [start_position[1] - goal_distance - 10, start_position[1] + goal_distance + 10]
            self.work_space_y = [start_position[1] - 28, start_position[0] + 28]
            self.work_space_z = [0, 3]
            self.max_episode_steps = 300
        else:
            raise Exception('Invalid env_name', self.env_name)

        self.client = self.dynamic_model.client
        self.state_feature_length = self.dynamic_model.state_feature_length
        self.cnn_feature_length = self.cfg.getint('options', 'cnn_feature_num')

        self.episode_num = 0
        self.total_step = 0
        self.step_num = 0
        self.cumulated_episode_reward = 0
        self.previous_distance_from_des_point = 0

        self.crash_distance = cfg.getfloat('environment', 'crash_distance')
        self.accept_radius = cfg.getint('environment', 'accept_radius')

        self.max_depth_meters = cfg.getint('environment', 'max_depth_meters')
        self.screen_height = cfg.getint('environment', 'screen_height')
        self.screen_width = cfg.getint('environment', 'screen_width')

        self.trajectory_list = []

        if self.perception_type == 'vetor':
            self.observation_space = spaces.Box(low=0, high=1,
                                                shape=(1, self.cnn_feature_length + self.state_feature_length),
                                                dtype=np.float32
                                                )
        else:
            self.observation_space = spaces.Box(low=0, high=255,
                                                shape=(self.screen_height, self.screen_width, 2),
                                                dtype=np.uint8
                                                )
        self.action_space = self.dynamic_model.action_space

        self.reward_type = None
        try:
            self.reward_type = cfg.get('options', 'reward_type')
            print('Reward type', self.reward_type)
        except NoOptionError:
            self.reward_type = None

    def reset(self):
        self.dynamic_model.reset()
        self.episode_num += 1
        self.step_num = 0
        self.cumulated_episode_reward = 0
        # self.dynamic_model.goal_distance = self.dynamic_model.get_distance_to_goal_2d()
        self.dynamic_model.goal_distance = self.dynamic_model.get_distance()
        self.previous_distance_from_des_point = self.dynamic_model.goal_distance

        self.trajectory_list = []

        obs = self.get_obs()

        return obs

    def step(self, action):
        distance_n = self.getdis()
        self.dynamic_model.set_action(action)
        position_ue4 = self.dynamic_model.get_position()
        self.trajectory_list.append(position_ue4)

        obs = self.get_obs()

        is_not_inside_workspace_now = self.is_not_inside_workspace()
        has_reached_des_pose = self.is_in_desired_pose()
        too_close_to_obstable = self.is_crashed()

        done = is_not_inside_workspace_now or has_reached_des_pose or too_close_to_obstable
        if self.step_num >= self.max_episode_steps:
            done = True
        if self.reward_type == 'reward_final':
            reward = self.compute_reward_final(done, action)
        if is_not_inside_workspace_now:
            reward = -100
        if has_reached_des_pose:
            reward = 1000
        if too_close_to_obstable:
            reward = -100

        self.cumulated_episode_reward += reward

        info = {
            'is_success': has_reached_des_pose,
            'is_crash': too_close_to_obstable,
            'is_not_in_workspace': is_not_inside_workspace_now,
            'step_num': self.step_num,
            'reward': self.cumulated_episode_reward
        }
        if done:
            print(info)

        self.print_train_info_airsim(action, obs, reward, position_ue4, distance_n)
        self.set_pyqt_signal_multirotor(action, reward)
        self.step_num += 1
        self.total_step += 1
        return obs, reward, done, info

    def get_obs(self):
        if self.perception_type == 'vector':
            obs = self.get_obs_vector()
        else:
            obs = self.get_obs_image()

        return obs

    def get_obs_image(self):
        # Normal mode: get depth image then transfer to matrix with state
        # 1. get current depth image and transfer to 0-255  0-20m 255-0m
        image = self.get_depth_image()  # 0-6550400.0 float 32
        image_resize = cv2.resize(image, (self.screen_width, self.screen_height))
        self.min_distance_to_obstacles = image.min()
        # switch 0 and 255
        image_scaled = np.clip(image_resize, 0, self.max_depth_meters) / self.max_depth_meters * 255
        image_scaled = 255 - image_scaled
        image_uint8 = image_scaled.astype(np.uint8)

        # 2. get current state (relative_pose, velocity)
        state_feature_array = np.zeros((self.screen_height, self.screen_width))
        state_feature = self.dynamic_model._get_state_feature()
        state_feature_array[0, 0:self.state_feature_length] = state_feature

        # 3. generate image with state
        image_with_state = np.array([image_uint8, state_feature_array])
        image_with_state = image_with_state.swapaxes(0, 2)
        image_with_state = image_with_state.swapaxes(0, 1)

        return image_with_state

    def get_depth_image(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthVis, True)
        ])

        # check observation
        while responses[0].width == 0:
            print("get_image_fail...")
            responses = self.client.simGetImages(
                airsim.ImageRequest("0", airsim.ImageType.DepthVis, True))

        depth_img = airsim.list_to_2d_float_array(
            responses[0].image_data_float, responses[0].width,
            responses[0].height)

        depth_meter = depth_img * 100

        return depth_meter

    def get_obs_vector(self):
        image = self.get_depth_image()  # 0-6550400.0 float 32
        self.min_distance_to_obstacles = image.min()

        image_scaled = np.clip(image, 0, self.max_depth_meters) / self.max_depth_meters * 255
        image_scaled = 255 - image_scaled
        image_uint8 = image_scaled.astype(np.uint8)

        image_obs = image_uint8
        split_row = 1
        split_col = 5

        v_split_list = np.vsplit(image_obs, split_row)

        split_final = []
        for i in range(split_row):
            h_split_list = np.hsplit(v_split_list[i], split_col)
            for j in range(split_col):
                split_final.append(h_split_list[j].max())

        img_feature = np.array(split_final) / 255.0

        state_feature = self.dynamic_model._get_state_feature() / 255

        feature_all = np.concatenate((img_feature, state_feature), axis=0)

        self.feature_all = feature_all

        feature_all = np.reshape(feature_all, (1, len(feature_all)))

        return feature_all

    def compute_reward_final(self, done, action):
        reward = 0
        distance_reward_coef = 200

        if not done:
            # distance_now = self.get_distance_to_goal_3d()
            distance_now = self.getdis()
            distance_d = self.previous_distance_from_des_point - distance_now
            reward_distance = distance_reward_coef * distance_d / self.dynamic_model.goal_distance
            self.previous_distance_from_des_point = distance_now

            current_pose = self.dynamic_model.get_position()
            goal_pose = self.dynamic_model.goal_position
            z = current_pose[2]
            z_g = goal_pose[2]
            dis = self.goal_distance - current_pose[0]

            punishment_xy = np.clip(dis / 10, 0, 1)
            punishment_z = 0.5 * np.clip((z - z_g) / 5, 0, 1)
            punishment_pose = punishment_xy + punishment_z

            if self.min_distance_to_obstacles < 10:
                punishment_obs = 1 - np.clip((self.min_distance_to_obstacles - self.crash_distance) / 5, 0, 1)
            else:
                punishment_obs = 0

            punishment_action = 0

            # add yaw_rate cost
            yaw_speed_cost = abs(action[-1]) / self.dynamic_model.yaw_rate_max_rad

            if self.dynamic_model.navigation_3d:
                # add action and z error cost
                v_z_cost = ((abs(action[1]) / self.dynamic_model.v_z_max) ** 2)
                z_err_cost = (
                        (abs(self.dynamic_model.state_raw[1]) / self.dynamic_model.max_vertical_difference) ** 2)
                punishment_action += (v_z_cost + z_err_cost)

            punishment_action += yaw_speed_cost

            yaw_error = self.dynamic_model.state_raw[2]
            yaw_error_cost = abs(yaw_error / 90)

            # reward = reward_distance - 0.1 * punishment_pose - 0.2 * punishment_obs \
            #         - 0.1 * punishment_action - 0.5 * yaw_error_cost
            reward = reward_distance * 1.5 - 0.1 * punishment_pose - 0.2 * punishment_obs \
                     - 0.1 * punishment_action - 0.8 * yaw_error_cost
        return reward

    def is_not_inside_workspace(self):
        is_not_inside = False
        current_position = self.dynamic_model.get_position()
        if current_position[0] < self.work_space_x[0] or current_position[0] > self.work_space_x[1] or \
                current_position[1] < self.work_space_y[0] or current_position[1] > self.work_space_y[1] or \
                current_position[2] < self.work_space_z[0] or current_position[2] > self.work_space_z[1]:
            is_not_inside = True

        return is_not_inside

    def is_in_desired_pose(self):
        in_desired_pose = False
        if self.getdis() < self.accept_radius:
            in_desired_pose = True

        return in_desired_pose

    def is_crashed(self):
        is_crashed = False
        collision_info = self.client.simGetCollisionInfo()
        if collision_info.has_collided or self.min_distance_to_obstacles < self.crash_distance:
            is_crashed = True

        return is_crashed

    def getdis(self):
        current_pose = self.dynamic_model.get_position()
        return self.goal_distance - current_pose[0]

    def print_train_info_airsim(self, action, obs, reward, pose, distance):
        # if self.perception_type == 'split' or self.perception_type == 'lgmd':
        #     feature_all = self.feature_all
        # elif self.perception_type == 'vector':
        #     feature_all = self.feature_all
        # else:
        #     if self.cfg.get('options', 'algo') == 'TD3' or self.cfg.get('options', 'algo') == 'SAC':
        #         feature_all = self.model.actor.features_extractor.feature_all
        #     elif self.cfg.get('options', 'algo') == 'PPO':
        #         feature_all = self.model.policy.features_extractor.feature_all

        # self.client.simPrintLogMessage('feature_all: ', str(feature_all))

        msg_train_info = "EP: {} Step: {} Total_step: {}".format(self.episode_num, self.step_num, self.total_step)
        self.client.simPrintLogMessage('Train: ', msg_train_info)
        self.client.simPrintLogMessage('Action: ', str(action))
        self.client.simPrintLogMessage('reward: ', "{:4.4f} total: {:4.4f}".format(
            reward, self.cumulated_episode_reward))
        # self.client.simPrintLogMessage('Feature_norm: ', str(self.dynamic_model.state_norm))
        # self.client.simPrintLogMessage('Feature_raw: ', str(self.dynamic_model.state_raw))
        self.client.simPrintLogMessage('Min_depth: ', str(self.min_distance_to_obstacles))
        self.client.simPrintLogMessage('position: ', str(pose))
        self.client.simPrintLogMessage('distance: ', str(distance))

    def set_pyqt_signal_multirotor(self, action, reward):
        step = int(self.total_step)

        state = self.dynamic_model.state_raw
        if self.dynamic_model.navigation_3d:
            action_output = action
            state_output = state
        else:
            action_output = np.array([action[0], 0, action[1]])
            state_output = np.array([state[0], 0, state[2], state[3], 0, state[5]])

        self.action_signal.emit(step, action_output)
        self.state_signal.emit(step, state_output)

        self.attitude_signal.emit(step, np.asarray(self.dynamic_model.get_attitude(
        )), np.asarray(self.dynamic_model.get_attitude_cmd()))
        self.reward_signal.emit(step, reward, self.cumulated_episode_reward)



