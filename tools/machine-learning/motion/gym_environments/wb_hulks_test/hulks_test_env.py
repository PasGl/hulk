import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import math
import struct
import time
from websocket import create_connection

# Code-snippet for Scilab-rl src/custom_envs/register_envs.py
#
#    register(
#    	id='wb-hulks-experiment-v0',
#     	entry_point='custom_envs.wb_hulks_experiment.hulks_test_env:NAOEnvMaker',
#     	kwargs=kwargs,
#     	max_episode_steps=2500,
#    )

# Calling Scilab-rl after Webots world walk_stabilization.wbt started
#
# python3 src/main.py n_epochs=100 wandb=0 algorithm=sac env=wb-hulks-experiment-v0

CACHED_ENV = None
ACTION_SIZE = 2 # adjust this if you want to include more joints (see step function)
FULL_ACTION_SIZE = 26
OBSERVATION_SIZE = (4 * 26) + (3 * 2) + (2 * 8) + 1 + (2 * 2)
FULL_OBSERVATION_SIZE = (4 * 26) + (3 * 2) + (2 * 8) + 1 + (2 * 2)


class NAOEnvMaker:
    def __new__(cls, *args, **kwargs):
        global CACHED_ENV
        if CACHED_ENV is None:
            return NAOEnv(*args, **kwargs)
        else:
            print('\033[92m' + 'Using cached Env' + '\033[0m')
            return CACHED_ENV


class NAOEnv(gym.GoalEnv):
    def __init__(self, render_mode='none', ik=0, reward_type='sparse'):
        print('\033[92m' + 'Creating new Env' + '\033[0m')
        global CACHED_ENV
        CACHED_ENV = self
        self.reward_type = reward_type
        self.delay = 0.01
        self.step_ctr = 0
        self.resets = 0
        self.previous_distance = 100
        self.start_time = time.time()
        self.fps_time = time.time()
        self.fps_counter = 0
        self.action_space = spaces.Box(-0.1, 0.1,
                                       shape=(ACTION_SIZE,), dtype='float32')
        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(-4.5, 4.5, shape=(2,), dtype='float32'),
            achieved_goal=spaces.Box(-4.5, 4.5, shape=(2,), dtype='float32'),
            observation=spaces.Box(-3., 3., shape=(OBSERVATION_SIZE-5,), dtype='float32'),))
        self.nao_websocket = create_connection("ws://localhost:9990")
        self.webots_supervisor_websocket = create_connection("ws://localhost:9980")
        self.fallen = False
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode='none', **kwargs):
        return

    def reset(self):
        super().reset()
        self.resets += 1
        self.fps_time = time.time()
        self.fps_counter = 0
        self.seed()
        action = np.array([0.0 for _ in range(ACTION_SIZE)])
        self.step_ctr = 0
        self.initial_info = {'initial_distance': 3.0, 'previous_distance': 3.0, 'fallen': 0.0}
        self.previous_distance = 3.0
        obs, r, done, info = self.step(action)
        if info['is_success']:
            print("Success !!!!")
        if self.fallen:
            print("Fallen :(")
        print("Resets:", self.resets)
        self.webots_supervisor_websocket.send_binary(b'0')
        self.webots_supervisor_websocket.recv()
        time.sleep(4.00)
        obs, r, done, info = self.step(action)
        self.initial_info = info
        self.initial_info["initial_distance"] = math.dist(obs["achieved_goal"], obs["desired_goal"])
        self.initial_info["previous_distance"] = math.dist(obs["achieved_goal"], obs["desired_goal"])
        self.initial_info["fallen"] = 0.0
        self.fallen = False
        self.previous_distance = math.dist(obs["achieved_goal"], obs["desired_goal"])
        return obs

    def compute_reward(self, achieved_goal, goal, info):
        if len(achieved_goal.shape) == 2:
            reward = [self.compute_reward(g1, g2, i) for g1, g2, i in zip(achieved_goal, goal, info)]
        else:
            if info["fallen"] == 1.0:
                reward =  -1.0
            elif math.dist(achieved_goal, goal) < 0.5:
                reward =  0.0
                #reward =  0.0
                #reward = 1.0 - (math.dist(achieved_goal, goal) / info["initial_distance"])
            else:
                #reward =  0.0
                reward =  -1.0
                #reward =  info["previous_distance"] - math.dist(achieved_goal, goal)
            #print(reward, info["previous_distance"], achieved_goal, goal)
        return np.array(reward)

    def step(self, action):
        full_action = [0.0 for _ in range(FULL_ACTION_SIZE)]
        full_action[12] = action[0] # left ankle pitch
        full_action[24] = action[1] # right ankle pitch
        #full_action[11] = action[2] # left knee pitch
        #full_action[23] = action[3] # right knee pitch
        #full_action[13] = action[4] # left ankle roll
        #full_action[25] = action[5] # right ankle roll
        #full_action[10] = action[6] # left hip pitch
        #full_action[22] = action[7] # right hip pitch
        #full_action[2]  = action[8] # left shoulder pitch
        #full_action[14] = action[9] # right shoulder pitch
        action_bin = struct.pack('%sf' % len(full_action), *full_action)
        
        self.nao_websocket.send_binary(action_bin)
        observation_bin = self.nao_websocket.recv()

        obs_unpacked = struct.unpack('%sf' % FULL_OBSERVATION_SIZE, observation_bin)

        #self.webots_supervisor_websocket.send_binary(b'1')
        #translations_bin = self.webots_supervisor_websocket.recv()
        #translations = struct.unpack('%sf' % 4, translations_bin)

        translations = obs_unpacked[26+26+26+26+2+2+2+8+8+1:]
        observation = list(obs_unpacked[:26+26+26+26+2+2+2+8+8])
        obs = {'observation': np.array(observation.copy()), 'achieved_goal': np.array([translations[0], translations[1]]),
               'desired_goal': np.array([translations[2], translations[3]]),
               'non_noisy_obs': observation.copy()}
        
        self.fps_counter += 1
        if time.time() - self.fps_time > 1:
            #print("FPS:", self.fps_counter)
            self.fps_time = time.time()
            self.fps_counter = 0

        is_success = 0
        done = False
        if self.initial_info is not None:
            if math.dist([translations[0], translations[1]], [translations[2], translations[3]]) < 0.5:
                is_success = 1
#                done = True
#            elif obs_unpacked[26+26+26+26+8+8] == 10.0:
#                done = True
        if obs_unpacked[26+26+26+26+2+2+2+8+8] == 10.0:
            fallen = 1.0
            self.fallen = True
        else:
            fallen = 0.0
        info = {'is_success': is_success, 
                'initial_distance': self.initial_info['initial_distance'],
                'previous_distance': self.previous_distance,
                'fallen': fallen}

        r = self.compute_reward(obs['achieved_goal'], obs['desired_goal'], info)
        self.previous_distance = math.dist(obs["achieved_goal"], obs["desired_goal"])
        self.step_ctr += 1
        return obs, r, done, info
