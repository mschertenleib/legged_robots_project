from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    """ Learn to move towards goal location. """

    dt = self._time_step

    # distance_to_goal, angle_to_goal = self.get_distance_and_angle_to_goal()

    base_pos = self.robot.GetBasePosition()
    base_vel = self.robot.GetBaseLinearVelocity()
    base_angular_vel = self.robot.GetBaseAngularVelocity()
    vel_x = base_vel[0]
    vel_y = base_vel[1]
    vel_z = base_vel[2]
    roll_pitch_rate = base_angular_vel[0:2]
    yaw_rate = base_angular_vel[2]
    goal_vec = self._goal_location - base_pos[0:2]

    desired_vel = 1.0 * unit_vector(goal_vec)
    desired_yaw_rate = 0.0  # -0.2 * angle_to_goal

    def f(x):
        return np.exp(-np.dot(x, x) / 0.25)

    reward_vel_x = 5.0 * dt * f(desired_vel[0] - vel_x)
    reward_vel_y = 5.0 * dt * f(desired_vel[1] - vel_y)
    reward_yaw_rate = 0.5 * dt * (desired_yaw_rate - yaw_rate)
    reward_vel_z = -2.0 * dt * vel_z ** 2
    reward_roll_pitch_rates = -0.05 * dt * np.dot(roll_pitch_rate, roll_pitch_rate)
    reward_work = 0.0
    if len(self._dt_motor_velocities) >= 2:
        motor_velocity_delta = self._dt_motor_velocities[-1] - self._dt_motor_velocities[-2]
        reward_work = -0.001 * dt * np.abs(np.dot(self._dt_motor_torques[-1], motor_velocity_delta))

    reward = (reward_vel_x
              + reward_vel_y
              + reward_yaw_rate
              + reward_vel_z
              + reward_roll_pitch_rates
              + reward_work)

    # return reward
    return max(reward, 0)  # keep rewards positive


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "spin"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "DEFAULT",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
