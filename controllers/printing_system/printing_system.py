import tempfile

from controller import Supervisor
from ikpy.chain import Chain
import numpy as np
from toolpath import Toolpath


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

    def __call__(self, current_time: float) -> np.array:
        if current_time < self.start_time + self.duration:
            return (self.start_time + self.duration -
                    current_time) / self.duration * self.start_angles + (
                        current_time -
                        self.start_time) / self.duration * self.end_angles
        else:
            return self.end_angles

    def is_finished(self, current_time):
        return current_time > self.start_time + self.duration


class Platform:

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

    def get_y_displacement(self):
        return self.get_translation()[2]

    def get_displacement_interpolator(self, starting_time: float,
                                      target_displacement: np.array,
                                      duration: float):
        return LinearInterpolator(starting_time, duration,
                                  self.get_y_displacement(),
                                  target_displacement)


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

    def get_joint_angle_interpolator(self, starting_time: float,
                                     target_position: np.array,
                                     duration: float):
        target_joint_angles = self.arm_chain.inverse_kinematics(
            target_position,
            max_iter=self.INVERSE_KINEMATICS_MAX_ITERATIONS,
            # This might be dangerous in the first step
            initial_position=self.get_joint_angles())
        return LinearInterpolator(starting_time, duration,
                                  self.get_joint_angles(), target_joint_angles)


def main():
    supervisor = Supervisor()
    time_step = int(4 * supervisor.getBasicTimeStep())
    supervisor.step(time_step)

    robotic_arm = RoboticArm(supervisor, time_step)
    platform = Platform(supervisor, time_step)
    # Starts with the intial positon of the robot
    g_code_commands = [
        GCodeCommand(np.array([1.65, 0.0, 1.76]), 1.0),
        GCodeCommand(np.array([1.65, 0.0, 0.1]), 1.0),
        GCodeCommand(np.array([2.0, -1.0, 0.1]), 0.2),
        GCodeCommand(np.array([1.4, 1.0, 0.1]), 0.2),
        GCodeCommand(np.array([2.0, -1.0, 0.1]), 0.2)
    ]
    toolpath = Toolpath.from_g_code_commands(g_code_commands)

    current_time = 0.0
    for set_point in toolpath:
        print(set_point)
        robotic_arm_interpolator = robotic_arm.get_joint_angle_interpolator(
            current_time, set_point.robotic_arm_position,
            set_point.robotic_arm_duration)
        platform_interpolator = platform.get_displacement_interpolator(
            current_time, set_point.platform_displacement,
            set_point.platform_duration)
        while supervisor.step(time_step) != -1:
            current_time += time_step / 1000.0
            if not robotic_arm_interpolator.is_finished(
                    current_time) or not platform_interpolator.is_finished(
                        current_time):
                robotic_arm_joint_angles = robotic_arm_interpolator(
                    current_time)
                platform_y_displacement = platform_interpolator(current_time)
            else:
                break
            robotic_arm.set_joint_angles(robotic_arm_joint_angles)
            platform.set_y_displacement(platform_y_displacement)


if __name__ == "__main__":
    main()
