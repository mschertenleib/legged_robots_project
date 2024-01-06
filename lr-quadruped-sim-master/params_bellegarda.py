from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    base_pos = self.robot.GetBasePosition()
    base_vel = self.robot.GetBaseLinearVelocity()
    vel_xy = base_vel[0:2]

    target_vel_xy = unit_vector(self._goal_location - base_pos[0:2])
    # target_vel_xy = np.array([1.0, 0.0])
    vel_forward = np.dot(target_vel_xy, vel_xy)
    vel_lateral = np.cross(target_vel_xy, vel_xy)

    # TODO: add other terms from slides, especially z position
    reward_forward = 2.0 * vel_forward
    reward_lateral = -2.0 * vel_lateral ** 2
    reward_energy = -0.008 * np.abs(
        np.dot(self.robot.GetMotorTorques(), self.robot.GetMotorVelocities())) * self._time_step

    reward = (reward_forward
              + reward_lateral
              + reward_energy)

    """print(
        f"{reward_forward:.4f} "
        f"{reward_lateral:.4f} "
        f"{reward_energy:.4f} ")"""

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "bellegarda"
env_config = {"motor_control_mode": "CARTESIAN_PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "DEFAULT",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
