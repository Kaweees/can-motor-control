"""Exercise the joint interface through the real native codec and router."""
from pathlib import Path

import numpy as np
import pytest

from can_motor_control import CanFrame, MockCanBus, MotorSpec, RobotBuilder
from can_motor_control.damiao import DamiaoCodec, MotorType
from can_motor_control.openyam import OpenYamJointInterface, gripper_feedforward


@pytest.fixture
def yam():
    bus = MockCanBus("follower_l")
    builder = RobotBuilder().add_bus("follower_l", bus, DamiaoCodec())
    robot = OpenYamJointInterface.add_to(builder, "follower_l").build()
    robot.connect()
    return robot, OpenYamJointInterface(robot, "follower_l"), bus


def golden(type_id, kp, kd, q, dq, tau):
    path = Path(__file__).parents[2] / "crates/damiao-codec/tests/fixtures/anvil_control.csv"
    for row in path.read_text().splitlines():
        c = row.split(",")
        if c[0] == "mit" and int(c[1]) == type_id:
            if np.allclose([float(v) for v in c[2:7]], [kp, kd, q, dq, tau], rtol=0, atol=1e-14):
                return bytes.fromhex(c[7])
    raise AssertionError("missing independent Anvil fixture")


@pytest.mark.parametrize("opening,motor_q,tau", [(0.025, -1.429, 0.8), (0.05, -2.858, 0), (-0.003, 0.17148, -0.8)])
def test_seven_joint_commands_match_anvil_frames(yam, opening, motor_q, tau):
    robot, joints, bus = yam
    joints.write([0.55] * 6 + [opening], kp=[40] * 6 + [0.1], kd=[3] * 6 + [0.04],
                 velocity=[0.2] * 6 + [123], effort=[0.3] * 6 + [tau])
    frames = bus.sent_frames()
    assert [f.id for f in frames] == [1, 2, 3, 4, 5, 6, 8]
    for i, frame in enumerate(frames[:6]):
        assert frame.payload() == golden(3 if i < 3 else 1, 40, 3, 0.55, 0.2, 0.3)
    assert frames[6].payload() == golden(1, 0.1, 0.04, motor_q, 0, tau)
    assert not any(f.is_fd() for f in frames)


def test_enable_uses_mit_without_gripper_calibration(yam):
    robot, joints, bus = yam
    joints.set_mode()
    robot.enable()
    frames = bus.sent_frames()
    assert len(frames) == 14
    assert [f.payload() for f in frames[:7]] == [bytes([i, 0, 0x55, 10, 1, 0, 0, 0]) for i in [1, 2, 3, 4, 5, 6, 8]]
    assert [f.id for f in frames[7:]] == [1, 2, 3, 4, 5, 6, 8]
    assert all(f.payload() == bytes.fromhex("fffffffffffffffc") for f in frames[7:])
    robot.disable()


def test_joint_feedback_matches_anvil_units_and_refresh(yam):
    robot, joints, bus = yam
    joints.refresh()
    assert [f.payload() for f in bus.sent_frames()] == [bytes([i, 0, 0xcc, 0, 0, 0, 0, 0]) for i in [1, 2, 3, 4, 5, 6, 8]]
    for i in [1, 2, 3, 4, 5, 6, 8]:
        bus.inject_frame(CanFrame.classical(i + 0x10, bytes([0x10 | i, 0x91, 0x33, 0x90, 9, 0, 30, 35])))
    robot.tick(0)
    q = 1.6798275730525667
    np.testing.assert_allclose(joints.positions(), [q] * 6 + [q * 0.05 / -2.858])
    np.testing.assert_allclose(joints.velocities(), [1.0021978021978022] * 3 + [3.7582417582417591] * 3 + [0])
    np.testing.assert_allclose(joints.efforts(), [3.5076923076923094] * 3 + [1.2527472527472518] * 3 + [0])
    # Raw motor telemetry remains available separately.
    assert robot["follower_l_gripper"].motor.velocity != 0


@pytest.mark.parametrize("field,value", [("position", [0] * 6), ("position", [0] * 6 + [np.nan]), ("effort", [0] * 6 + [np.inf]), ("kd", [0] * 8)])
def test_invalid_command_is_rejected_before_any_motor_is_commanded(yam, field, value):
    _, joints, bus = yam
    args = dict(position=[0] * 7, kp=[1] * 7, kd=[0.1] * 7)
    args[field] = value
    with pytest.raises(ValueError):
        joints.write(**args)
    assert bus.sent_frames() == []


@pytest.mark.parametrize("mistake", ["order", "model", "calibration"])
def test_joint_interface_rejects_incompatible_motor_mapping(mistake):
    bus = MockCanBus("follower_l")
    motors = [MotorSpec(f"follower_l_joint{i}", MotorType.DM4340 if i <= 3 or mistake == "model" else MotorType.DM4310, i, i + 0x10) for i in range(1, 7)]
    if mistake == "order":
        motors.reverse()
    robot = (RobotBuilder().add_bus("follower_l", bus, DamiaoCodec())
             .add_arm("follower_l", bus="follower_l", motors=motors)
             .add_gripper("follower_l_gripper", bus="follower_l",
                          motor=MotorSpec("follower_l_finger_joint1", MotorType.DM4310, 8, 0x18),
                          opening_direction="decreasing_position" if mistake == "calibration" else None)
             .build())
    with pytest.raises(ValueError):
        OpenYamJointInterface(robot, "follower_l")
    assert bus.sent_frames() == []


def test_gripper_feedforward_direction_and_transition():
    assert gripper_feedforward(0.025, 0.025) == 0.0
    assert gripper_feedforward(0.05, 0.0) == pytest.approx(-0.8)
    assert gripper_feedforward(0.0, 0.05) == pytest.approx(0.8)
    assert gripper_feedforward(0.0, 0.003) == pytest.approx(0.7241186029158932)
    with pytest.raises(ValueError):
        gripper_feedforward(0, 0, error_threshold=0)


def test_dual_arms_route_identical_can_ids_on_separate_buses():
    builder = RobotBuilder()
    buses = {name: MockCanBus(name) for name in ("follower_l", "follower_r")}
    for name, bus in buses.items():
        builder = builder.add_bus(name, bus, DamiaoCodec())
        builder = OpenYamJointInterface.add_to(builder, name)
    robot = builder.build()
    robot.connect()
    buses["follower_l"].inject_frame(CanFrame.classical(0x11, bytes.fromhex("11ffffffffff1e23")))
    buses["follower_r"].inject_frame(CanFrame.classical(0x11, bytes.fromhex("1100000000001e23")))
    robot.tick(0)
    assert OpenYamJointInterface(robot, "follower_l").positions()[0] == 12.5
    assert OpenYamJointInterface(robot, "follower_r").positions()[0] == -12.5
