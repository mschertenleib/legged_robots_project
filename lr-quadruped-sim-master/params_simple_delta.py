from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    vel_z = self.robot.GetBaseLinearVelocity()[2]
    roll_pitch = self.robot.GetBaseOrientationRollPitchYaw()[0:2]

    distance_to_goal, angle_to_goal = self.get_distance_and_angle_to_goal()

    reward_forward = 1.0 * (self._prev_pos_to_goal - distance_to_goal) / self._time_step
    reward_roll_pitch = -5.0 * np.dot(roll_pitch, roll_pitch)
    reward_z = -4.0 * vel_z ** 2
    reward_energy = 0.0
    for tau, vel in zip(self._dt_motor_torques, self._dt_motor_velocities):
        reward_energy += np.abs(np.dot(tau, vel)) * self._time_step
    reward_energy *= -0.01

    reward = (reward_forward
              + reward_roll_pitch
              + reward_z
              + reward_energy)

    """print(
        f"{reward_forward:.4f} "
        f"{reward_roll_pitch:.4f} "
        f"{reward_z:.4f} "
        f"{reward_energy:.4f} ")"""

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "simple_delta_default"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "DEFAULT",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
