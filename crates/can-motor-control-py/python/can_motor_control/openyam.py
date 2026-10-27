"""OpenYAM joint commands matching Anvil's OpenArm_v10HW motor boundary.

Joint order is J1..J6 (radians), then finger_joint1 (metres). Gains and effort
commands remain in motor units. Scheduling, homing and dynamics are supplied
by the caller, as ROS controllers supply them to Anvil's hardware interface.
"""
from __future__ import annotations

import math
import numpy as np
from numpy.typing import ArrayLike, NDArray

from can_motor_control import Arm, Gripper, MotorSpec, Robot, RobotBuilder
from can_motor_control.damiao import MotorType

# config/openyam_inference.yaml in anvil-loader; pass explicitly to write().
INFERENCE_KP = (40.0, 65.0, 65.0, 25.0, 12.0, 12.0, 0.1)
INFERENCE_KD = (3.0, 4.5, 4.5, 0.7, 0.2, 0.2, 0.04)


def gripper_feedforward(
    target: float, measured: float, *, magnitude: float = 0.8, error_threshold: float = 0.003
) -> float:
    """Anvil DynamicsController's gripper effort, in motor Nm.

    Positions and threshold are in metres. The smooth effort saturates toward
    ``magnitude``; its sign follows the gripper's negative motor-to-joint ratio.
    Supply fresh measured feedback and pass the result as ``effort[6]`` to write.
    """
    if not all(math.isfinite(v) for v in (target, measured, magnitude, error_threshold)):
        raise ValueError("gripper feedforward inputs must be finite")
    if magnitude < 0 or error_threshold <= 0:
        raise ValueError("magnitude must be nonnegative and error_threshold positive")
    return magnitude * math.tanh((measured - target) * (1.5 / error_threshold))


def _vector(value: ArrayLike, name: str) -> NDArray[np.float64]:
    result = np.array(value, dtype=np.float64, copy=True)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain seven finite values in J1..J6, gripper order")
    return result


class OpenYamJointInterface:
    """Seven-joint MIT interface using Anvil's fixed motor and gripper mapping.

    Use ``add_to`` to configure a builder, then construct this view of its
    robot. ``set_mode`` must run after connect and before enable. ``write``
    sends commands; call ``robot.tick`` to receive feedback. No automatic
    gripper calibration or encoder zeroing is performed.
    """

    @staticmethod
    def add_to(builder: RobotBuilder, name: str, *, bus: str | None = None) -> RobotBuilder:
        """Add six arm motors and one gripper to an already declared CAN bus.

        ``bus`` defaults to ``name`` (usually ``follower_l`` or ``follower_r``).
        The groups are named ``name`` and ``name + '_gripper'``.
        """
        bus = name if bus is None else bus
        motors = [
            MotorSpec(f"{name}_joint{i}", MotorType.DM4340 if i <= 3 else MotorType.DM4310,
                      send_id=i, recv_id=0x10 + i)
            for i in range(1, 7)
        ]
        return builder.add_arm(name, bus=bus, motors=motors).add_gripper(
            f"{name}_gripper", bus=bus,
            motor=MotorSpec(f"{name}_finger_joint1", MotorType.DM4310, send_id=8, recv_id=0x18),
        )

    def __init__(self, robot: Robot, name: str) -> None:
        arm = robot[name]
        gripper = robot[f"{name}_gripper"]
        if not isinstance(arm, Arm) or len(arm) != 6 or not isinstance(gripper, Gripper):
            raise ValueError("OpenYAM requires a six-motor arm and a one-motor gripper")
        for i in range(1, 7):
            motor = arm.motor_at(i - 1)
            if (motor.name, motor.send_id, motor.recv_id, motor.motor_type) != (
                f"{name}_joint{i}", i, i + 0x10, ("damiao", 3 if i <= 3 else 1)
            ):
                raise ValueError(f"incorrect OpenYAM motor mapping at joint {i}")
        motor = gripper.motor
        if (motor.send_id, motor.recv_id, motor.motor_type) != (8, 0x18, ("damiao", 1)):
            raise ValueError("incorrect OpenYAM gripper motor mapping")
        if gripper.has_opening_control:
            raise ValueError("OpenYAM uses fixed gripper mapping; omit opening_direction")
        self._robot = robot  # Keep the native handles' owner alive.
        self._arm = arm
        self._gripper = gripper

    def set_mode(self) -> None:
        """Select MIT mode for this arm and gripper before enabling them."""
        self._arm.set_mode("mit")
        self._gripper.set_mode("mit")

    def write(
        self,
        position: ArrayLike,
        *,
        kp: ArrayLike,
        kd: ArrayLike,
        velocity: ArrayLike = (0.0,) * 7,
        effort: ArrayLike = (0.0,) * 7,
    ) -> None:
        """Send seven joint commands using Anvil's hardware-interface semantics.

        ``position`` is six angles in radians followed by a finger position in
        metres (0 closed, 0.05 open). Negative finger positions such as -0.003
        are preserved, allowing the same squeeze command as Anvil. ``kp``,
        ``kd`` and ``effort`` pass through in motor units, including the gripper.
        Its commanded velocity is always zero, matching Anvil. All inputs are
        validated before the first frame is sent. Finite MIT fields saturate
        to their protocol limits in the codec.
        """
        q = _vector(position, "position")
        p = _vector(kp, "kp")
        d = _vector(kd, "kd")
        dq = _vector(velocity, "velocity")
        tau = _vector(effort, "effort")
        # Preserve Anvil's multiplication/division order and do not clip to
        # the nominal mechanical range. The encoder clamps to the wire range.
        with np.errstate(over="ignore"):
            q[6] = q[6] * -2.858 / 0.05
        if not np.isfinite(q[6]):
            raise ValueError("gripper position overflows motor radians")
        dq[6] = 0.0
        commands = np.column_stack((p, d, q, dq, tau))
        self._arm.mit_control(commands[:6])
        self._gripper.mit_control(*commands[6])

    def refresh(self) -> None:
        """Request feedback without commanding motion; then call robot.tick."""
        self._arm.refresh()
        self._gripper.refresh()

    def positions(self) -> NDArray[np.float64]:
        """Cached J1..J6 radians and finger position in metres, without clipping."""
        return np.append(self._arm.positions(), self._gripper.motor.position * 0.05 / -2.858)

    def velocities(self) -> NDArray[np.float64]:
        """Cached arm rad/s and zero gripper joint velocity, matching Anvil."""
        return np.append(self._arm.velocities(), 0.0)

    def efforts(self) -> NDArray[np.float64]:
        """Cached arm Nm and zero gripper joint effort, matching Anvil."""
        return np.append(self._arm.torques(), 0.0)
