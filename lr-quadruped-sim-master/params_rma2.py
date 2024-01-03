from env.quadruped_gym_env import *


def reward_flag_run(self: QuadrupedGymEnv):
    """ Learn to move towards goal location. """

    # Get state
    base_pos = self.robot.GetBasePosition()
    base_vel = self.robot.GetBaseLinearVelocity()
    base_angle = self.robot.GetBaseOrientationRollPitchYaw()
    base_angular_vel = self.robot.GetBaseAngularVelocity()
    _, _, feet_normal_forces, _ = self.robot.GetContactInfo()

    vel_xy = base_vel[0:2]
    vel_z = base_vel[2]
    roll_pitch = base_angle[0:2]
    yaw_rate = base_angular_vel[2]
    goal_vec = self._goal_location - base_pos[0:2]
    vel_forward = np.dot(unit_vector(goal_vec), vel_xy)
    vel_lateral = np.cross(unit_vector(goal_vec), vel_xy)

    reward_forward = 20.0 * vel_forward
    reward_lateral = -21.0 * vel_lateral ** 2
    reward_work = 0.0
    if len(self._dt_motor_angles) >= 2:
        motor_angle_delta = self._dt_motor_angles[-1] - self._dt_motor_angles[-2]
        reward_work = -0.002 * np.abs(np.dot(self._dt_motor_torques[-1], motor_angle_delta))
    reward_roll_pitch = -1.5 * np.dot(roll_pitch, roll_pitch)
    reward_velocity_z = -2.0 * vel_z ** 2

    reward = (reward_forward
              + reward_lateral
              + reward_work
              + reward_roll_pitch
              + reward_velocity_z)

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "rma2"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "LR_COURSE_OBS",
              "test_env": False,
              "reward_flag_run": reward_flag_run}
