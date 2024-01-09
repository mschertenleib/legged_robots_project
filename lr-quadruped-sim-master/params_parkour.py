from env.quadruped_gym_env import *


# From https://robot-parkour.github.io/resources/Robot_Parkour_Learning.pdf

def reward_flag_run(self: QuadrupedGymEnv):
    """ Learn to move towards goal location. """

    # distance_to_goal, angle_to_goal = self.get_distance_and_angle_to_goal()

    # base_pos = self.robot.GetBasePosition()
    base_vel = self.robot.GetBaseLinearVelocity()
    base_angular_vel = self.robot.GetBaseAngularVelocity()
    vel_x = base_vel[0]
    vel_y = base_vel[1]
    yaw_rate = base_angular_vel[2]

    # TODO: goal tracking
    target_vel_x = 1.0

    reward_forward = -1.0 * np.abs(vel_x - target_vel_x)
    reward_lateral = -1.0 * vel_y ** 2
    reward_yaw_rate = 0.05 * np.exp(-np.abs(yaw_rate))
    power = self.robot.GetMotorTorques() * self.robot.GetMotorVelocities()
    reward_energy = -1e-5 * np.dot(power, power)

    reward = (reward_forward
              + reward_lateral
              + reward_yaw_rate
              + reward_energy)

    return reward


LEARNING_ALG = "PPO"
LOG_DIR_NAME = "parkour"
env_config = {"motor_control_mode": "PD",
              "task_env": "FLAGRUN",
              "observation_space_mode": "LR_COURSE_OBS",
              "test_env": False,
              "reward_flag_run": reward_flag_run}

# NOTE: the observation space includes:
# - Motor angles
# - Motor velocities
# - Base roll and pitch
# - Last action
