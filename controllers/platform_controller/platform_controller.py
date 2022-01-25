import tempfile

from controller import Supervisor
from ikpy.chain import Chain
import numpy as np


class SetPoint:

    def __init__(self, position, duration) -> None:
        self.position = position
        self.duration = duration


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
            # Why i+1 ?
            self.motors[i].setPosition(joint_angles[i + 1])

    def follow_trajectory(self, trajectory: list[SetPoint], current_time):
        INVERSE_KINEMATICS_MAX_ITERATIONS = 4

        target_joint_angles = self.arm_chain.inverse_kinematics(
            trajectory[0].position,
            max_iter=INVERSE_KINEMATICS_MAX_ITERATIONS,
            initial_position=self.get_joint_angles())
        interpolator = LinearInterpolator(0.0, trajectory[0].duration,
                                          self.starting_position,
                                          target_joint_angles)
        joint_angles = interpolator.interpolate_joint_angles(current_time)
        self.set_joint_angles(joint_angles)

    def get_joint_angle_interpolator(self, starting_time, set_point: SetPoint):
        target_joint_angles = self.arm_chain.inverse_kinematics(
            set_point.position,
            max_iter=self.INVERSE_KINEMATICS_MAX_ITERATIONS,
            # This might be dangerous in the first step
            initial_position=self.get_joint_angles())
        return LinearInterpolator(starting_time, set_point.duration,
                                  self.get_joint_angles(), target_joint_angles)


def main():
    # supervisor = Supervisor()
    # time_step = int(4 * supervisor.getBasicTimeStep())

    # robotic_arm = RoboticArm(supervisor, time_step)
    # trajectory = [
    #     SetPoint(np.array([1.65, -0.5, 1.76]), 10.0),
    #     SetPoint(np.array([1.65, 1.0, 1.76]), 10.0),
    #     SetPoint(np.array([1.65, -1.0, 1.76]), 10.0)
    # ]
    time_step = 32
    robot = Supervisor()
    motor = robot.getDevice('sliding_joint')
    t = 0.0

    while robot.step(time_step) != -1:
        position = 10 * t
        motor.setPosition(position)
        t += time_step / 1000.0
    # current_time = 0.0
    # for set_point in trajectory:
    #     interpolator = robotic_arm.get_joint_angle_interpolator(
    #         current_time, set_point)
    #     while supervisor.step(time_step) != -1:
    #         current_time += time_step / 1000.0
    #         if not interpolator.is_finished(current_time):
    #             joint_angles = interpolator.interpolate_joint_angles(current_time)
    #         else:
    #             break
    #         robotic_arm.set_joint_angles(joint_angles)


# TODO:
# - [X] move the robotic arm in its own world
# - [ ] add the moving platform using a sliding joint
# - [ ] implement the synchronization algorithm
# - [ ] visualize the trajectory (optional)

if __name__ == "__main__":
    main()
