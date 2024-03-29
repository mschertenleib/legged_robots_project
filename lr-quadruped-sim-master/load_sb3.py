# SPDX-FileCopyrightText: Copyright (c) 2022 Guillaume Bellegarda. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2022 EPFL, Guillaume Bellegarda

import importlib
import os, sys
import gym
import numpy as np
import time
import matplotlib
import matplotlib.pyplot as plt
from sys import platform
# may be helpful depending on your system
# if platform =="darwin": # mac
#   import PyQt5
#   matplotlib.use("Qt5Agg")
# else: # linux
#   matplotlib.use('TkAgg')

import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

# stable-baselines3
from stable_baselines3.common.monitor import load_results 
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3 import PPO, SAC
# from stable_baselines3.common.cmd_util import make_vec_env
from stable_baselines3.common.env_util import make_vec_env # fix for newer versions of stable-baselines3

from env.quadruped_gym_env import QuadrupedGymEnv
# utils
from utils.utils import plot_results
from utils.file_utils import get_latest_model, load_all_results

PARAMS_FROM_FILE = True
PARAMS_FILE = "params_simple_yaw"
if PARAMS_FROM_FILE:
    params = importlib.import_module(PARAMS_FILE)
    LEARNING_ALG = params.LEARNING_ALG
    LOG_DIR_NAME = params.LOG_DIR_NAME
    env_config = params.env_config
else:
    LEARNING_ALG = "PPO"
    LOG_DIR_NAME = "PPO_PD_FLAGRUN_DEFAULT"
    # initialize env configs (render at test time)
    # check ideal conditions, as well as robustness to UNSEEN noise during training
    env_config = {"motor_control_mode": "PD",
                  "task_env": "LR_COURSE_TASK",
                  "observation_space_mode": "LR_COURSE_OBS",
                  "test_env": False,
                  "reward_flag_run": None}

interm_dir = "./logs/intermediate_models/"
# path to saved models, i.e. interm_dir + '121321105810'
log_dir = interm_dir + LOG_DIR_NAME
GRAVITY = 9.81

env_config['render'] = True
env_config['record_video'] = False
env_config['add_noise'] = True
env_config['competition_env'] = False

# get latest model and normalization stats, and plot 
stats_path = os.path.join(log_dir, "vec_normalize.pkl")
model_name = get_latest_model(log_dir)
print("model_name", model_name)

monitor_results = load_results(log_dir)
print(monitor_results)
if len(monitor_results)>0:
    plot_results([log_dir] , 10e10, 'timesteps', LEARNING_ALG + ' ')
plt.show() 

# reconstruct env 
env = lambda: QuadrupedGymEnv(**env_config)
env = make_vec_env(env, n_envs=1)
env = VecNormalize.load(stats_path, env)
env.training = False    # do not update stats at test time
env.norm_reward = False # reward normalization is not needed at test time

# load model
if LEARNING_ALG == "PPO":
    model = PPO.load(model_name, env)
elif LEARNING_ALG == "SAC":
    model = SAC.load(model_name, env)
print("\nLoaded model", model_name, "\n")

obs = env.reset()
episode_reward = 0


velocity = []
CoT = []
foot_pos = []


for i in range(200):
    action, _states = model.predict(obs,deterministic=False)  # sample at test time? ([TODO]: test)
    # print('action', action.shape)
    obs, rewards, dones, info = env.step(action)
    episode_reward += rewards
    if dones:
        print('episode_reward', episode_reward)
        print('Final base position', info[0]['base_pos'])
        episode_reward = 0
    # Velocity calculation
    velocity.append(env.envs[0].env.robot.GetBaseLinearVelocity()[0:2])
    # CoT calculation
    masses = 0
    for mass in env.envs[0].env.robot.GetTotalMassFromURDF():
      masses += mass
    Torque_abs = np.abs(env.envs[0].env.robot.GetMotorTorques())
    Velocity = env.envs[0].env.robot.GetMotorVelocities()
    cot_local = np.dot(Torque_abs,np.abs(Velocity))/(np.linalg.norm(Torque_abs) * masses * GRAVITY)
    CoT.append(cot_local)
    # Foot trajectory
    foot_pos.append(env.envs[0].env.robot.GetMotorAngles())

velocity = np.array(velocity)
foot_pos = np.array(foot_pos)

fig, ax = plt.subplots()
ax.plot(velocity[:, 0], label='vx')
ax.plot(velocity[:, 1], label='vy')
ax.legend()
plt.xlabel('Time [samples]')
plt.ylabel('Velocity [m/s]')
plt.show()

plt.figure()
plt.plot(CoT)
plt.xlabel('Time [samples]')
plt.ylabel('CoT')
plt.show()

fig, axs = plt.subplots(3, 1)
fig.suptitle('Joint Angles')
titles = ['Hip', 'Thigh', 'Calf']
legs = ['FR', 'FL', 'RR', 'RL']
for i in range(3):
    ax = axs[i]
    ax.set(ylabel=titles[i] + ' [rad]')
    if i < 2:
        ax.tick_params('x', labelbottom=False)
    else:
        ax.set(xlabel='Time [samples]')
    for j in range(4):
        ax.plot(foot_pos[:, j * 3 + i], label=legs[j])
    ax.legend()
plt.tight_layout()
plt.show()
