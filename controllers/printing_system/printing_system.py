import tempfile

from controller import Supervisor
from ikpy.chain import Chain
import numpy as np


class GCodeCommand:

    def __init__(self, position: np.array, speed: float) -> None:
        self.position = position  # Relative to the robotic arm base
        self.speed = speed  # [m/s]

    def get_xy_radius(self):
        return np.linalg.norm(self.position[:2])


class LinearInterpolator:

    def __init__(self, start_time, duration, start_angles, end_angles) -> None:
        self.start_time = start_time
        self.duration = duration
        self.start_angles = start_angles
        self.end_angles = end_angles

    def interpolate_joint_angles(self, current_time: float) -> np.array:
        if current_time < self.start_time + self.duration:
            return (self.start_time + self.duration -
                    current_time) / self.duration * self.start_angles + (
                        current_time -
                        self.start_time) / self.duration * self.end_angles
        else:
            return self.end_angles

    def is_finished(self, current_time):
        return current_time > self.start_time + self.duration


class MovingPlatform:

    def __init__(self, supervisor, time_step) -> None:
        self.supervisor = supervisor
        self.time_step = time_step
        self.translation_field = self.supervisor.getFromDef(
            "MOVING_PLATFORM").getField('translation')
        self.translation = self.get_translation()
        self.lower_limit = -2.0
        self.upper_limit = 2.0

    def get_translation(self):
        return self.translation_field.getSFVec3f()

    def set_y_displacement(self, y_displacement: float):
        if self.lower_limit <= y_displacement and y_displacement <= self.upper_limit:
            current_position = self.get_translation()
            self.translation_field.setSFVec3f(
                [current_position[0], current_position[1], y_displacement])


class RoboticArm:

    def __init__(self, supervisor, time_step) -> None:
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(supervisor.getUrdf().encode('utf-8'))
        self.arm_chain = Chain.from_urdf_file(filename)
        for i in [0, 6]:
            self.arm_chain.active_links_mask[i] = False

        self.motors = []
        for link in self.arm_chain.links:
            if 'motor' in link.name:
                motor = supervisor.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(time_step)
                self.motors.append(motor)
        self.INVERSE_KINEMATICS_MAX_ITERATIONS = 4
        self.arm_base = supervisor.getSelf()
        self.initial_joint_angles = self.get_joint_angles()

    def get_joint_angles(self) -> np.array:
        return np.array(
            [0] + [m.getPositionSensor().getValue()
                   for m in self.motors] + [0])

    def set_joint_angles(self, joint_angles: np.array):
        for i in range(len(self.motors)):
            self.motors[i].setPosition(joint_angles[i + 1])

    def get_joint_angle_interpolator(self, starting_time,
                                     target_position: np.array,
                                     duration: float):
        target_joint_angles = self.arm_chain.inverse_kinematics(
            target_position.position,
            max_iter=self.INVERSE_KINEMATICS_MAX_ITERATIONS,
            # This might be dangerous in the first step
            initial_position=self.get_joint_angles())
        return LinearInterpolator(starting_time, duration,
                                  self.get_joint_angles(), target_joint_angles)


class PrinitingSystemSetPoint:

    def __init__(self, robotic_arm_position: np.array,
                 robotic_arm_duration: float,
                 moving_platform_displacement: float,
                 moving_platform_duration: float) -> None:
        self.robotic_arm_position = robotic_arm_position
        self.robotic_arm_duration = robotic_arm_duration
        self.moving_platform_position = moving_platform_displacement
        self.moving_platform_duration = moving_platform_duration


class Toolpath:

    def __init__(self, trajectory) -> None:
        self.trajectory = trajectory

    @classmethod
    def from_g_code_commands(cls, g_code_commands: list[GCodeCommand],
                             working_radius: float):
        # Assume for now that the set points are really close
        trajectory = []
        for i in range(1, len(g_code_commands)):
            if g_code_commands[i].get_xy_radius() < working_radius:
                trajectory.append(
                    PrinitingSystemSetPoint(g_code_commands[i].position,
                                            g_code_commands[i].speed, 0.0,
                                            g_code_commands[i].speed))
            else:
                robotic_arm_target_position = np.array([
                    g_code_commands[i].position[0], 0.0,
                    g_code_commands[i].position[1]
                ])
                robotic_arm_speed = g_code_commands[i].speed * (
                    g_code_commands[i].position[1] -
                    g_code_commands[i - 1].position[1]
                ) / np.linalg.norm(g_code_commands[i].position[:2] -
                                   g_code_commands[i - 1].position[:2])
                moving_platform_displacement = g_code_commands[i].position[1]

                moving_platform_speed = g_code_commands[i].speed * (
                    g_code_commands[i].position[0] -
                    g_code_commands[i - 1].position[0]
                ) / np.linalg.norm(g_code_commands[i].position[:2] -
                                   g_code_commands[i - 1].position[:2])

                trajectory.append(
                    PrinitingSystemSetPoint(robotic_arm_target_position,
                                            robotic_arm_speed,
                                            moving_platform_displacement,
                                            moving_platform_speed))


def main():
    supervisor = Supervisor()
    time_step = int(4 * supervisor.getBasicTimeStep())

    robotic_arm = RoboticArm(supervisor, time_step)
    moving_platform = MovingPlatform(supervisor, time_step)
    # Starts with the intial positon of the robot
    g_code_commands = [
        GCodeCommand(np.array([1.65, 0.0, 1.76]), 2.0),
        GCodeCommand(np.array([1.65, -0.5, 1.76]), 10.0),
        GCodeCommand(np.array([1.65, 1.0, 1.76]), 10.0),
        GCodeCommand(np.array([1.65, -1.0, 1.76]), 10.0)
    ]
    toolpath = Toolpath.from_g_code_commands(g_code_commands, 5.0)
    for set_point in toolpath:
        print(set_point)

    # current_time = 0.0
    # for i in range(1, len(g_code_commands)):
    #     duration = np.linalg.norm(
    #         g_code_commands[i].position -
    #         g_code_commands[i - 1].position) / g_code_commands[i].speed
    #     target_position = g_code_commands[i].position

    #     interpolator = robotic_arm.get_joint_angle_interpolator(
    #         current_time, target_position, duration)
    #     while supervisor.step(time_step) != -1:
    #         current_time += time_step / 1000.0
    #         moving_platform.get_translation()
    #         moving_platform.set_y_displacement(
    #             0.2 * np.sin(current_time * 2 * np.pi))
    #         if not interpolator.is_finished(current_time):
    #             joint_angles = interpolator.interpolate_joint_angles(
    #                 current_time)
    #         else:
    #             break
    #         robotic_arm.set_joint_angles(joint_angles)


# TODO:
# - [X] move the robotic arm in its own world
# - [X] add the moving platform using a sliding joint
# - [X] add the projection on the working envelope
# - [ ] handle the case where the speed for the platform is 0.0
# - [ ] handle the case where a set point is exactly on the working radius -> do not add a second set point
# - [ ] normal movement until projected point!
# - [ ] the robotic arm has to stay on the edge of the working envelope during movement of the platform!
# - [ ] printing system set points still contain a duartion and not a speed
# - [ ] keep track if the working radius is left and if so save it in a boolean variable
# - [ ] swap the axis the paper 
# - [ ] include orientation information
# - [ ] implement the synchronization algorithm
# - [ ] visualize the trajectory (optional)

if __name__ == "__main__":
    main()
