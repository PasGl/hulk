import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import math
import random
import signal
import struct
import time
from websocket import create_connection

# Code-snippet for Scilab-rl src/custom_envs/register_envs.py
#
#    register(
#    	id='wb-hulks-experiment-v0',
#     	entry_point='custom_envs.wb_hulks_experiment.hulks_test_env:NAOEnvMaker',
#     	kwargs=kwargs,
#     	max_episode_steps=5000,
#    )

# Calling Scilab-rl after Webots world walk_stabilization.wbt started
#
# python3 src/main.py n_epochs=100 wandb=0 algorithm=sac env=wb-hulks-experiment-v0

CACHED_ENV = None
ACTION_SIZE = 2 # adjust this if you want to include more joints (see step function)
FULL_ACTION_SIZE = 26
OBSERVATION_SIZE = (2 * 26) +  (2 * 8) + 1
FULL_OBSERVATION_SIZE = (2 * 26) + (2 * 8) + 1


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
        self.reward_type = reward_type
        self.delay = 0.01
        self.fps = 0
        self.fps_count = 0
        self.step_ctr = 0
        self.resets = 0
        self.previous_distance = 100
        self.start_time = time.time()
        self.nao_websocket = None #create_connection("ws://localhost:9990")
        self.webots_supervisor_websocket = create_connection("ws://localhost:9980")
        global CACHED_ENV
        CACHED_ENV = self
        self.action_space = spaces.Box(-0.2, 0.2,
                                       shape=(ACTION_SIZE,), dtype='float32')
        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(0., 10., shape=(1,), dtype='float32'),
            achieved_goal=spaces.Box(0., 10., shape=(1,), dtype='float32'),
            observation=spaces.Box(-3., 3., shape=(OBSERVATION_SIZE-1,), dtype='float32'),))
        self.initial_obs = None
        self.nao_websocket = create_connection("ws://localhost:9990")
        self.webots_supervisor_websocket = create_connection("ws://localhost:9980")
        self.initial_obs = self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode='none', **kwargs):
        return

    def reset(self):
        super().reset()
        self.resets += 1
        self.seed()
        action = np.array([0.0 for _ in range(ACTION_SIZE)])
        self.step_ctr = 0
        self.previous_distance = 100
        obs, r, done, info = self.step(action)
        if info['is_success']:
            print("Success !!!!")
        print("Resets:", self.resets)
        self.webots_supervisor_websocket.send("reset")
        time.sleep(4.00)
        obs, r, done, info = self.step(action)
        self.initial_obs = obs
        return obs

    def compute_reward(self, achieved_goal, goal, info):
        if self.previous_distance == 100 or achieved_goal[0] == 10:
            self.previous_distance = achieved_goal[0]
            return 0.0
        else:
            old_distance = self.previous_distance
            self.previous_distance = achieved_goal[0]
            #return (old_distance - achieved_goal[0])
            #return (old_distance - achieved_goal[0])/achieved_goal[0] #non-sparse 1
            #return (old_distance - achieved_goal[0])/math.sqrt(self.step_ctr+1)
            #return 1/achieved_goal[0]
            #if achieved_goal[0] < 0.2:
            #    return 1.0/((self.step_ctr+1)/2000)
            #else:
            #    return 0.0
            if achieved_goal[0] < 0.2:
                return self.initial_obs['achieved_goal'][0]/((self.step_ctr+1)/1000)
            else:
                return 0.0

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
 
        obs = struct.unpack('%sf' % FULL_OBSERVATION_SIZE, observation_bin)
        obs = {'observation': np.array(obs[:26+26+8+8]), 'achieved_goal': np.array([obs[26+26+8+8]]),
               'desired_goal': np.array([0.0]),
               'non_noisy_obs': obs[:26+26+8+8]}
        is_success = 0
        done = False
        if self.initial_obs is not None:
            if obs['achieved_goal'][0] < 0.2:
                is_success = 1
                done = True
            elif obs['achieved_goal'][0] == 10.0:
                done = True

        info = {'is_success': is_success}

        r = self.compute_reward(obs['achieved_goal'], obs['desired_goal'], info)
        self.step_ctr += 1
        return obs, r, done, info
