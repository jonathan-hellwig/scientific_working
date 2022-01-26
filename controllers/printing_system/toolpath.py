import numpy as np
import unittest


class GCodeCommand:

    def __init__(self, position: np.array, speed: float) -> None:
        self.position = position  # Relative to the robotic arm base
        self.speed = speed  # [m/s]

    def get_xy_radius(self):
        return np.linalg.norm(self.position[:2])


class PrinitingSystemSetPoint:

    def __init__(self, robotic_arm_position: np.array,
                 robotic_arm_duration: float,
                 moving_platform_displacement: float,
                 moving_platform_duration: float) -> None:
        self.robotic_arm_position = robotic_arm_position
        self.robotic_arm_duration = robotic_arm_duration
        self.moving_platform_position = moving_platform_displacement
        self.moving_platform_duration = moving_platform_duration

    def __str__(self):
        return f'PrinitingSystemSetPoint({self.robotic_arm_position},{self.robotic_arm_duration}, {self.moving_platform_position},{self.moving_platform_duration})'


def project_to_working_envelope(first_point, second_point, working_radius):
    scaling = (first_point[0] *
               (first_point[0] - second_point[0]) + first_point[1] *
               (first_point[1] - second_point[1]) + np.sqrt(
                   (first_point[0] *
                    (first_point[0] - second_point[0]) + first_point[1] *
                    (first_point[1] - second_point[1]))**2 +
                   ((first_point[0] - second_point[0])**2 +
                    (first_point[1] - second_point[1])**2) *
                   (working_radius**2 - first_point[0]**2 - first_point[1]**2))
               ) / ((first_point[0] - second_point[0])**2 +
                    (first_point[1] - second_point[1])**2)
    return first_point + scaling * (second_point - first_point)


def calculate_printing_system_set_point(first_position, second_position,
                                        speed):
    robotic_arm_target_position = np.array(
        [second_position[0], 0.0, second_position[2]])
    robotic_arm_speed = speed * (second_position[0] - first_position[0]
                                 ) / np.linalg.norm(second_position[:2] -
                                                    first_position[:2])
    moving_platform_displacement = second_position[1]

    moving_platform_speed = speed * (second_position[1] - first_position[1]
                                     ) / np.linalg.norm(second_position[:2] -
                                                        first_position[:2])

    return robotic_arm_target_position, robotic_arm_speed, moving_platform_displacement, moving_platform_speed


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
                                            0.0))
            else:
                robotic_arm_target_position, robotic_arm_speed, moving_platform_displacement, moving_platform_speed = calculate_printing_system_set_point(
                    g_code_commands[i - 1].position,
                    g_code_commands[i].position, g_code_commands[i].speed)

                trajectory.append(
                    PrinitingSystemSetPoint(robotic_arm_target_position,
                                            robotic_arm_speed,
                                            moving_platform_displacement,
                                            moving_platform_speed))
        return cls(trajectory)

    def __getitem__(self, index):
        return self.trajectory[index]


class TestToolpath(unittest.TestCase):

    def test_toolpath_creation(self):
        working_radius = 4.0
        g_code_commands = [
            GCodeCommand(np.array([0.0, 0.0, 0.0]), 1.0),
            GCodeCommand(np.array([0.0, 1.0, 0.0]), 2.0),
            GCodeCommand(np.array([0.0, 2.0, 0.0]), 3.0),
            GCodeCommand(np.array([0.0, 3.0, 0.0]), 4.0),
            GCodeCommand(np.array([0.0, 4.0, 0.0]), 5.0),
            GCodeCommand(np.array([0.0, 5.0, 0.0]), 6.0),
        ]

        toolpath = Toolpath.from_g_code_commands(g_code_commands, 5.0)
        for set_point in toolpath:
            print(set_point)


class TestProjection(unittest.TestCase):

    def test_project_to_working_envelope_one(self):
        first_point = np.array([0.0, 0.5])
        second_point = np.array([0.0, 1.5])
        expected_projection = np.array([0.0, 1.0])
        working_radius = 1.0
        self.assertTrue(
            np.allclose(
                project_to_working_envelope(first_point, second_point,
                                            working_radius),
                expected_projection))

    def test_project_to_working_envelope_two(self):
        first_point = np.array([0.0, 0.25])
        second_point = np.array([0.0, 2.0])
        expected_projection = np.array([0.0, 1.0])
        working_radius = 1.0
        self.assertTrue(
            np.allclose(
                project_to_working_envelope(first_point, second_point,
                                            working_radius),
                expected_projection))

    def test_project_to_working_envelope_three(self):
        first_point = np.array([0.5, 0.0])
        second_point = np.array([1.5, 0.0])
        expected_projection = np.array([1.0, 0.0])
        working_radius = 1.0
        self.assertTrue(
            np.allclose(
                project_to_working_envelope(first_point, second_point,
                                            working_radius),
                expected_projection))

    def test_project_to_working_envelope_four(self):
        first_point = np.array([0.5, 0.5])
        second_point = np.array([1.5, 1.5])
        expected_projection = np.array([1.0 / np.sqrt(2), 1.0 / np.sqrt(2)])
        working_radius = 1.0
        self.assertTrue(
            np.allclose(
                project_to_working_envelope(first_point, second_point,
                                            working_radius),
                expected_projection))


def main():

    # g_code_commands = [
    #     GCodeCommand(np.array([1.65, 0.0, 1.76]), 2.0),
    #     GCodeCommand(np.array([1.65, -0.5, 1.76]), 10.0),
    #     GCodeCommand(np.array([1.65, 1.0, 1.76]), 10.0),
    #     GCodeCommand(np.array([1.65, -1.0, 1.76]), 10.0)
    # ]
    # toolpath = Toolpath.from_g_code_commands(g_code_commands, 5.0)
    # for set_point in toolpath:
    #     print(set_point)
    test = TestToolpath()
    test.test_toolpath_creation()


if __name__ == "__main__":
    main()
