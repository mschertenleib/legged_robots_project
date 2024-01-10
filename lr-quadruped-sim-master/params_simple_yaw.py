from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    base_vel = self.robot.GetBaseLinearVelocity()
    base_orientation = self.robot.GetBaseOrientationRollPitchYaw()
    vel_xy = base_vel[0:2]
    vel_z = base_vel[2]
    roll_pitch = base_orientation[0:2]
    yaw = base_orientation[2]

    target_vel_xy = unit_vector(np.array([-0.5, 1.0]))
    vel_forward = np.dot(target_vel_xy, vel_xy)
    vel_lateral = np.cross(target_vel_xy, vel_xy)
    orientation_xy = np.array([np.cos(yaw), np.sin(yaw)])
    angle_to_target = angle_between(orientation_xy, target_vel_xy)

    reward_forward = 1.0 * vel_forward
    reward_lateral = -1.0 * vel_lateral ** 2
    reward_roll_pitch = -5.0 * np.dot(roll_pitch, roll_pitch)
    reward_yaw = -3.0 * angle_to_target ** 2
    reward_z = -4.0 * vel_z ** 2
    reward_energy = 0.0
    for tau, vel in zip(self._dt_motor_torques, self._dt_motor_velocities):
        reward_energy += np.abs(np.dot(tau, vel)) * self._time_step
    reward_energy *= -0.01

    reward = (reward_forward
              + reward_lateral
              + reward_roll_pitch
              + reward_yaw
              + reward_z
              + reward_energy)

    """print(
        f"{reward_forward:.4f} "
        f"{reward_lateral:.4f} "
        f"{reward_roll_pitch:.4f} "
        f"{reward_z:.4f} "
        f"{reward_energy:.4f} ")"""

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "simple_diagonal_yaw"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "DEFAULT",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
