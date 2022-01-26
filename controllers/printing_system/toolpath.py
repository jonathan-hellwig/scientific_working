import numpy as np


class GCodeCommand:

    def __init__(self, position: np.array, speed: float) -> None:
        self.position = position  # Relative to the robotic arm base
        self.speed = speed  # [m/s]

    def get_xy_radius(self):
        return np.linalg.norm(self.position[:2])


class PrinitingSystemSetPoint:

    def __init__(self, robotic_arm_position: np.array,
                 robotic_arm_duration: float, platform_displacement: float,
                 platform_duration: float) -> None:
        self.robotic_arm_position = robotic_arm_position
        self.robotic_arm_duration = robotic_arm_duration
        self.platform_displacement = platform_displacement
        self.platform_duration = platform_duration

    def __str__(self):
        return f'PrinitingSystemSetPoint({self.robotic_arm_position},{self.robotic_arm_duration}, {self.platform_displacement},{self.platform_duration})'


def calculate_printing_system_set_point(first_position, second_position,
                                        speed):
    if not np.array_equal(first_position, second_position) and speed != 0.0:
        robotic_arm_target_position = np.array(
            [second_position[0], 0.0, second_position[2]])
        robotic_arm_speed = speed * np.sqrt(
            ((second_position[0] - first_position[0])**2 +
             (second_position[2] - first_position[2])**
             2)) / np.linalg.norm(second_position - first_position)

        platform_displacement = -second_position[1]

        platform_speed = speed * (second_position[1] - first_position[1]
                                  ) / np.linalg.norm(second_position -
                                                     first_position)

        if robotic_arm_speed != 0.0:
            robotic_arm_duration = np.linalg.norm(
                second_position - first_position) / np.abs(robotic_arm_speed)
            if platform_speed != 0:
                platform_duration = np.linalg.norm(
                    second_position - first_position) / np.abs(platform_speed)
            else:
                platform_duration = robotic_arm_duration
        else:
            platform_duration = np.linalg.norm(
                second_position - first_position) / np.abs(platform_speed)
            robotic_arm_duration = platform_duration
        duration = min(robotic_arm_duration, platform_duration)
        return PrinitingSystemSetPoint(robotic_arm_target_position, duration,
                                       platform_displacement, duration)
    else:
        quit()


class Toolpath:

    def __init__(self, trajectory: list[PrinitingSystemSetPoint]) -> None:
        self.trajectory = trajectory

    @classmethod
    def from_g_code_commands(cls, g_code_commands: list[GCodeCommand]):
        trajectory = []
        for i in range(1, len(g_code_commands)):
            set_point = calculate_printing_system_set_point(
                g_code_commands[i - 1].position, g_code_commands[i].position,
                g_code_commands[i].speed)
            trajectory.append(set_point)

        return cls(trajectory)

    def __getitem__(self, index):
        return self.trajectory[index]
