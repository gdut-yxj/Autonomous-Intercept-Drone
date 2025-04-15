import gym
import numpy as np
import torch
import os
import ast
import datetime
import gym_env.airsim_env
from configparser import ConfigParser
from stable_baselines3 import TD3, PPO
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from custom_policy import CNN_FC, CNN_GAP, CNN_GAP_BN, No_CNN, CNN_MobileNet, CNN_GAP_new
import sys
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(CURRENT_DIR))

HOME_PATH = os.getcwd()
print(HOME_PATH)

method = 'pure_rl'      # 1-pure_rl 2-generate_expert_data 3-bc_rl 4-offline_rl
policy = 'no_cnn'       # 1-cnn_fc 2-cnn_gap 3-no_cnn 4-cnn_mobile
env_name = 'forest'      # 1-trees  2-cylinder
algo = 'td3'            # 1-ppo 2-td3
action_num = '3d'       # 2d or 3d

noise_type = 'NA'
goal_distance = 70
noise_intensity = 0.1
gamma = 0.99
learning_rate = 5e-4
total_steps = 3e5


class RunTrain(object):
    def __init__(self, config):
        self.cfg = ConfigParser()
        self.cfg.read(config)
        self.env_name = self.cfg.get('options', 'env_name')
        self.project_name = self.env_name
        self.env = gym.make('airsim_env_v0')
        self.env.set_config(self.cfg)

    def run(self):
        print("run training")
        now = datetime.datetime.now()
        now_string = now.strftime('%Y_%m_%d_%H_%M_')
        file_path = 'logs/' + self.project_name + '/' + now_string + '_' + self.cfg.get('options', 'dynamic_name') \
                    + '_' + self.cfg.get('options', 'policy_name') + '_' + self.cfg.get('options', 'algo')
        log_path = file_path + '/tb_logs'
        model_path = file_path + '/models'
        config_path = file_path + '/config'
        data_path = file_path + '/data'
        os.makedirs(log_path, exist_ok=True)
        os.makedirs(model_path, exist_ok=True)
        os.makedirs(config_path, exist_ok=True)
        os.makedirs(data_path, exist_ok=True)

        with open(config_path + '/config.ini', 'w') as configfile:
            self.cfg.write(configfile)

        feature_num_state = self.env.dynamic_model.state_feature_length
        feature_num_cnn = self.cfg.getint('options', 'cnn_feature_num')
        policy_name = self.cfg.get('options', 'policy_name')

        if self.cfg.get('options', 'activation_function') == 'tanh':
            activation_function = torch.nn.Tanh
        else:
            activation_function = torch.nn.ReLU

        if policy_name == 'mlp':
            policy_base = 'MlpPolicy'
            policy_kwargs = dict(activation_fn=activation_function)
        else:
            policy_base = 'CnnPolicy'
            if policy_name == 'CNN_FC':
                policy_used = CNN_FC
            elif policy_name == 'CNN_GAP':
                policy_used = CNN_GAP_new
            elif policy_name == 'CNN_GAP_BN':
                policy_used = CNN_GAP_BN
            elif policy_name == 'CNN_MobileNet':
                policy_used = CNN_MobileNet
            elif policy_name == 'No_CNN':
                policy_used = No_CNN
            else:
                raise Exception('policy select error: ', policy_name)

            policy_kwargs = dict(
                features_extractor_class=policy_used,
                features_extractor_kwargs=dict(features_dim=feature_num_state + feature_num_cnn,
                                               state_feature_dim=feature_num_state),
                activation_fn=activation_function
            )

        net_arch_list = ast.literal_eval(self.cfg.get("options", "net_arch"))
        policy_kwargs['net_arch'] = net_arch_list

        algo = self.cfg.get('options', 'algo')
        print('algo: ', algo)
        if algo == 'PPO':
            model = PPO(
                policy_base,
                self.env,
                # n_steps = 200,
                learning_rate=self.cfg.getfloat('DRL', 'learning_rate'),
                policy_kwargs=policy_kwargs,
                tensorboard_log=log_path,
                seed=0,
                verbose=2
            )
        elif algo == 'TD3':
            # The noise objects for TD3
            n_actions = self.env.action_space.shape[-1]
            noise_sigma = self.cfg.getfloat(
                'DRL', 'action_noise_sigma') * np.ones(n_actions)
            action_noise = NormalActionNoise(mean=np.zeros(n_actions),
                                             sigma=noise_sigma)
            model = TD3(
                policy_base,
                self.env,
                action_noise=action_noise,
                learning_rate=self.cfg.getfloat('DRL', 'learning_rate'),
                gamma=self.cfg.getfloat('DRL', 'gamma'),
                policy_kwargs=policy_kwargs,
                learning_starts=self.cfg.getint('DRL', 'learning_starts'),
                batch_size=self.cfg.getint('DRL', 'batch_size'),
                train_freq=(self.cfg.getint('DRL', 'train_freq'), 'step'),
                gradient_steps=self.cfg.getint('DRL', 'gradient_steps'),
                buffer_size=self.cfg.getint('DRL', 'buffer_size'),
                tensorboard_log=log_path,
                seed=0,
                verbose=2
            )
        else:
            raise Exception('Invalid algo name : ', algo)

        # TODO create eval_callback
        # eval_freq = self.cfg.getint('TD3', 'eval_freq')
        # n_eval_episodes = self.cfg.getint('TD3', 'n_eval_episodes')
        # eval_callback = EvalCallback(self.env, best_model_save_path= file_path + '/eval',
        #                      log_path= file_path + '/eval', eval_freq=eval_freq, n_eval_episodes=n_eval_episodes,
        #                      deterministic=True, render=False)

        print('start training model')
        total_timesteps = self.cfg.getint('options', 'total_timesteps')
        self.env.model = model
        self.env.data_path = data_path

        model.learn(total_timesteps)
        model_name = 'model'
        model.save(model_path + '/' + model_name)

        print('training finished')
        print('model saved to: {}'.format(model_path))


def main():
    config_file = 'configs/SimpleMultirotor.ini'
    print(config_file)
    train = RunTrain(config_file)
    train.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('system exit')