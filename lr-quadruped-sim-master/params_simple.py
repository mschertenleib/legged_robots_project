from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    des_vel_x = 2.0
    # track the desired velocity
    vel_tracking_reward = 2.0 * np.exp(-1 / 0.25 * (self.robot.GetBaseLinearVelocity()[0] - des_vel_x) ** 2)
    # minimize yaw (go straight)
    yaw_reward = -0.2 * np.abs(self.robot.GetBaseOrientationRollPitchYaw()[2])
    # don't drift laterally
    drift_reward = -0.01 * abs(self.robot.GetBasePosition()[1])
    # minimize energy
    energy_reward = 0.0
    for tau, vel in zip(self._dt_motor_torques, self._dt_motor_velocities):
        energy_reward += np.abs(np.dot(tau, vel)) * self._time_step

    reward = (vel_tracking_reward
              + yaw_reward
              + drift_reward
              - 0.01 * energy_reward
              - 0.1 * np.linalg.norm(self.robot.GetBaseOrientation() - np.array([0, 0, 0, 1])))

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "simple"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "LR_COURSE_OBS",
              "test_env": False,
              "reward_flag_run": reward_flag_run}

# NOTE: the observation space includes:
# - Motor angles
# - Motor velocities,
# - Base roll and pitch,
# - Binary feet contacts
