from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    base_pos = self.robot.GetBasePosition()
    base_vel = self.robot.GetBaseLinearVelocity()
    base_orientation = self.robot.GetBaseOrientation()

    reward_velocity = 0.1 * base_vel[0]
    reward_energy = -0.008 * np.abs(
        np.dot(self.robot.GetMotorTorques(), self.robot.GetMotorVelocities())) * self._time_step
    reward_orientation = -0.1 * np.linalg.norm(base_orientation - np.array([0, 0, 0, 1]))
    reward_drift = -0.1 * np.abs(base_pos[1])
    reward_height = -0.1 * np.abs(base_pos[2] - 0.35)

    reward = (reward_velocity
              + reward_energy
              + reward_orientation
              + reward_drift
              + reward_height)

    """print(
        f"{reward_forward:.4f} "
        f"{reward_lateral:.4f} "
        f"{reward_energy:.4f} ")"""

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "bellegarda_straight"
env_config = {"motor_control_mode": "CARTESIAN_PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "DEFAULT",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
