"""Offline OpenYAM joint-command example; prints frames sent to MockCanBus.

Run from the repository root: uv run examples/07_openyam_joint_space.py
See docs/openyam.md for hardware setup and the motor/joint interface contract.
"""
from can_motor_control import MockCanBus, RobotBuilder
from can_motor_control.damiao import DamiaoCodec
from can_motor_control.openyam import INFERENCE_KD, INFERENCE_KP, OpenYamJointInterface


def main():
    bus = MockCanBus("follower_l")
    builder = RobotBuilder().add_bus("follower_l", bus, DamiaoCodec())
    robot = OpenYamJointInterface.add_to(builder, "follower_l").build()
    joints = OpenYamJointInterface(robot, "follower_l")
    robot.connect()
    try:
        joints.set_mode()
        robot.enable()
        before = len(bus.sent_frames())
        # Six angles in radians, then finger position in metres.
        joints.write([0, 0.55, 0.573, 0.2, 0, 0, 0.025],
                     kp=INFERENCE_KP, kd=INFERENCE_KD)
        robot.tick(0)
        for frame in bus.sent_frames()[before:]:
            print(f"CAN 0x{frame.id:02x}: {frame.payload().hex()}")
    finally:
        robot.disable()


if __name__ == "__main__":
    main()
