# OpenYAM joint control and Anvil compatibility

`can_motor_control.openyam.OpenYamJointInterface` implements the joint-to-motor
boundary of Anvil's `control/OpenArm_v10HW`. It accepts seven values in Anvil
order: J1–J6 in radians, then `finger_joint1` in metres. It sends MIT commands
through the native Rust codec and reads feedback through `Robot.tick`.

## Reference and comparison

The reference is the installed stack used by `/home/anvil/anvil-loader`, at
loader revision `5611ee2c11a355be128ffe23da050d4796cc39e7`. The implementation
lives in the ROS image at `/workspace/install/control/lib/libcontrol.so`;
the host loader supplies configuration and launches it. The motor mapping is
in `/home/anvil/anvil-loader/yam_description/urdf/yam.ros2_control.xacro` and the
generated `/home/anvil/anvil-loader/config/robot_description.urdf`.

The comparison started from upstream `can-motor-control` revision
`ef5c082b94d77fbfbf50b4a73a41dca44f578acf`. The fixes below follow the installed
Anvil version, including its DM4340 limits. The committed
`crates/damiao-codec/tests/fixtures/` directory records the image digest, library
hash, capture harness, and outputs from that library.

| Behavior | Previous target behavior | Anvil behavior, now implemented |
| --- | --- | --- |
| DM4340 velocity scale | ±10 rad/s | ±8 rad/s; DM4340_48V remains ±10 |
| MIT quantization | Round to nearest | Clamp, divide by range, multiply by integer maximum, truncate |
| Finite MIT values outside wire range | Reject command | Saturate each field to its encoded range |
| Feedback scaling | DM4340 limits for every motor | Decode using the configured model for each receive ID |
| Feedback containing position byte `0x33` or `0x55` | Drop as parameter traffic | Check the parameter ID header too |
| Feedback shorter than three bytes | Index panic | Ignore incomplete frames |
| Motor status | Discard status nibble | Preserve telemetry while reporting enable state and fault codes |
| Gripper joint commands | Raw radians, or generic normalized PosForce opening | Fixed metre-to-radian mapping with MIT commands |
| Gripper joint feedback | Raw motor quantities | Metre position; joint velocity and effort reported as zero |

The existing `06_read_robot_motor.py --robot openyam` already had the correct
motor IDs and models. The new reusable joint interface and TOML profile use
that same layout:

| Joint | Model | Send ID | Receive ID |
| --- | --- | --- | --- |
| J1–J3 | DM4340 | `0x01`–`0x03` | `0x11`–`0x13` |
| J4–J6 | DM4310 | `0x04`–`0x06` | `0x14`–`0x16` |
| Gripper | DM4310 | `0x08` | `0x18` |

Anvil's gripper conversion is `motor_q = joint_q * -2.858 / 0.05`. Thus 0 m is
closed, 0.025 m maps to −1.429 rad, and 0.05 m is open at −2.858 rad. The
conversion preserves negative squeeze targets: −0.003 m maps to +0.17148 rad.
These are finger-joint metres, as in Anvil's command topic. Gains and effort
pass through in motor units; the gripper velocity command is zero.

## Try it offline

From the repository root:

```bash
uv sync
uv run examples/07_openyam_joint_space.py
uv run --with pytest pytest -q tests/python/test_openyam.py
```

The example uses `MockCanBus` and prints the seven outgoing command frames.
The tests compare actual native output with Anvil's captured bytes.

## Use the hardware interface

Use a calibrated OpenYAM with the encoder zero points already used by Anvil.
The CAN interface must be available to this process and have a single active
controller. `configs/openyam_single.toml` selects classical CAN on `follower_l`.
For the other arm, replace the interface and name prefix with `follower_r`.

Initialize the robot and its joint view:

```python
from can_motor_control import Robot
from can_motor_control.openyam import (
    INFERENCE_KP, INFERENCE_KD, OpenYamJointInterface, gripper_feedforward,
)

robot = Robot.from_config("configs/openyam_single.toml")
joints = OpenYamJointInterface(robot, "follower_l")
robot.connect()
joints.set_mode()          # MIT for all seven motors, before enable
robot.enable()             # No endpoint discovery or encoder zeroing
```

Each iteration of your control loop sends joint commands and receives feedback:

```python
# q_target: six desired angles (rad), then desired finger position (m).
# dq_target: six desired angular velocities; the seventh entry is ignored.
# tau: six feedforward motor torques from your dynamics controller, then
#      gripper torque. Use fresh measured feedback for the gripper calculation.
tau[6] = gripper_feedforward(q_target[6], q_measured[6])
joints.write(q_target, velocity=dq_target, effort=tau,
             kp=INFERENCE_KP, kd=INFERENCE_KD)
robot.tick(per_bus_deadline_us=1000)
q_measured = joints.positions()
```

Use `joints.refresh(); robot.tick(...)` when polling without a motion command.
`positions()`, `velocities()`, and `efforts()` read cached feedback; `tick()`
alone does not request new telemetry. Arrange `robot.disable()` in a `finally`
block around your enabled loop. For two arms, add two buses and call
`OpenYamJointInterface.add_to(builder, "follower_l")` and the corresponding
`"follower_r"` call before building the robot.

`gripper_feedforward` reproduces the installed DynamicsController's smooth
gripper effort: `0.8 * tanh((measured - target) * (1.5 / 0.003))` in motor Nm.
The magnitude and error threshold are configurable. This effort matters at
Anvil's gripper gain of 0.1, which truncates to zero in the 12-bit KP encoding.
`write()` accepts explicit effort so callers can reproduce Anvil's hardware
input without double-applying a controller's feedforward term.

## Scope and API changes

This interface reproduces the motor/joint boundary. Anvil's ROS stack also
runs a 500 Hz controller manager, gravity compensation, gain ramping, homing,
trajectory handling, and hardware-state control. Those remain separate
controller responsibilities. The constants exported here come from
`config/openyam_inference.yaml`; passing a position alone does not reproduce
the entire ROS control stack.

The generic `Gripper.set_opening()` API keeps its normalized 0–1, discovered
endpoint, PosForce semantics. The OpenYAM adapter requires fixed mapping and
rejects groups configured with `opening_direction`. Raw motor feedback remains
available through `robot["follower_l_gripper"].motor`.

Rust callers decoding Damiao frames directly must use
`MotorCodec::decode_for_motor(frame, motor_ref)` and handle `Event::Feedback`.
Context-free `decode()` rejects state frames because the wire does not identify
the motor model. Other codecs keep their existing `decode()` implementation
through the trait's default contextual method. The runtime supplies the context
automatically. NaN and infinite MIT inputs are rejected rather than quantized.

Offline tests establish protocol and mapping parity for the recorded inputs.
Hardware motion, mechanical limits, and timing have not been validated by
these tests.
