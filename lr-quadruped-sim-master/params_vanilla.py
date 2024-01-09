from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    """ Learn to move towards goal location. """
    curr_dist_to_goal, angle = self.get_distance_and_angle_to_goal()

    # minimize distance to goal (we want to move towards the goal)
    dist_reward = 10 * (self._prev_pos_to_goal - curr_dist_to_goal)
    # minimize yaw deviation to goal (necessary?)
    yaw_reward = 0  # -0.01 * np.abs(angle)

    # minimize energy
    energy_reward = 0
    for tau, vel in zip(self._dt_motor_torques, self._dt_motor_velocities):
        energy_reward += np.abs(np.dot(tau, vel)) * self._time_step

    reward = (dist_reward
              + yaw_reward
              - 0.001 * energy_reward)

    return max(reward, 0)  # keep rewards positive


LEARNING_ALG = "SAC"
LOG_DIR_NAME = "vanilla_joint_SAC"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "DEFAULT",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
