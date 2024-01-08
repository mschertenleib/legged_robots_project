from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    base_pos = self.robot.GetBasePosition()
    roll_pitch = self.robot.GetBaseOrientationRollPitchYaw()[0:2]

    distance_to_goal, angle_to_goal = self.get_distance_and_angle_to_goal()

    reward_forward = 1.0 * (self._prev_pos_to_goal - distance_to_goal) / self._time_step
    reward_energy = -0.008 * np.abs(
        np.dot(self.robot.GetMotorTorques(), self.robot.GetMotorVelocities())) * self._time_step
    reward_orientation = -0.1 * np.linalg.norm(roll_pitch)
    reward_height = -0.1 * np.abs(base_pos[2] - 0.35)

    reward = (reward_forward
              + reward_energy
              + reward_orientation
              + reward_height)

    """print(
        f"{reward_forward:.4f} "
        f"{reward_lateral:.4f} "
        f"{reward_energy:.4f} ")"""

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "bellegarda_flagrun_delta"
env_config = {"motor_control_mode": "CARTESIAN_PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "LR_COURSE_OBS",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
