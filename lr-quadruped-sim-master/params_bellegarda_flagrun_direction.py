from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    base_pos = self.robot.GetBasePosition()
    vel_xy = self.robot.GetBaseLinearVelocity()[0:2]
    roll_pitch = self.robot.GetBaseOrientationRollPitchYaw()[0:2]

    target_vel_xy = unit_vector(self._goal_location - base_pos[0:2])
    vel_forward = np.dot(target_vel_xy, vel_xy)
    vel_lateral = np.cross(target_vel_xy, vel_xy)

    reward_forward = 1.0 * vel_forward
    reward_drift = -0.1 * np.abs(vel_lateral)
    reward_energy = -0.008 * np.abs(
        np.dot(self.robot.GetMotorTorques(), self.robot.GetMotorVelocities())) * self._time_step
    reward_orientation = -0.1 * np.linalg.norm(roll_pitch)
    reward_height = -0.1 * np.abs(base_pos[2] - 0.35)

    reward = (reward_forward
              + reward_drift
              + reward_energy
              + reward_orientation
              + reward_height)

    """print(
        f"{reward_forward:.4f} "
        f"{reward_lateral:.4f} "
        f"{reward_energy:.4f} ")"""

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "bellegarda_direction_ppo"
env_config = {"motor_control_mode": "CARTESIAN_PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "LR_COURSE_OBS",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
