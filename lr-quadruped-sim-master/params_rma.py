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
    reward_yaw_rate = -21.0 * yaw_rate ** 2
    reward_work = 0.0
    reward_ground_impact = 0.0
    reward_smoothness = 0.0
    if len(self._dt_motor_velocities) >= 2:
        motor_velocity_delta = self._dt_motor_velocities[-1] - self._dt_motor_velocities[-2]
        reward_work = -0.002 * self._time_step * np.abs(np.dot(self._dt_motor_torques[-1], motor_velocity_delta))
        feet_force_delta = self._dt_feet_forces[-1] - self._dt_feet_forces[-2]
        reward_ground_impact = -0.02 * np.dot(feet_force_delta, feet_force_delta)
        torque_delta = self._dt_motor_torques[-1] - self._dt_motor_torques[-2]
        reward_smoothness = -0.001 * np.dot(torque_delta, torque_delta)
    reward_roll_pitch = -1.5 * np.dot(roll_pitch, roll_pitch)
    reward_velocity_z = -2.0 * vel_z ** 2

    reward = (reward_forward
              + reward_lateral
              + reward_yaw_rate
              + reward_work
              + reward_ground_impact
              + reward_smoothness
              + reward_roll_pitch
              + reward_velocity_z)

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "rma"
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
