# Anvil control fixtures

`anvil_control.csv` contains outputs from the actual library installed by the
local `/home/anvil/anvil-loader` deployment, captured on 2026-10-27. The Rust
codec is never used to generate expected values.

- Loader checkout: `5611ee2c11a355be128ffe23da050d4796cc39e7` (local configuration retained).
- Docker image: `sha256:a003bbe7e8bd19af3eedc08b2757b64fd4cd8e40f87ebfdda1acf9bf71359f72`.
- Library in that image: `/workspace/install/control/lib/libcontrol.so`.
- Library SHA-256: `49eb990b2000ee07295a550cce4f44ad1dc8d9daca7e984e2f4126b52d3d4fae`.

`capture_anvil.cpp` invokes the exported pure MIT encoder and feedback decoder
and reads the exported motor-limit table. Its ABI declarations were checked
against this library's DWARF types. The Motor constructor only initializes
data; the harness never constructs a CAN device or ROS controller. These
declarations are specific to this image, not a supported Anvil C++ API.

The CSV covers all 13 motor types: 13 limit rows, 104 MIT command rows, and 39
feedback rows. Commands include zero, signed setpoints, endpoints, values beyond
the wire ranges, and gripper setpoints. Feedback includes position bytes `0x33`
and `0x55`, which previously caused valid telemetry to be dropped.

Reproduce from the repository root on an x86-64 host with this image available:

```bash
mkdir -p /tmp/anvil-codec-capture
g++ -std=c++17 -O0 crates/damiao-codec/tests/fixtures/capture_anvil.cpp \
  -ldl -o /tmp/anvil-codec-capture/probe
docker run --rm --network none --read-only --cap-drop ALL \
  -v /tmp/anvil-codec-capture:/probe:ro --entrypoint /bin/bash \
  sha256:a003bbe7e8bd19af3eedc08b2757b64fd4cd8e40f87ebfdda1acf9bf71359f72 \
  -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && /probe/probe' \
  > /tmp/anvil-codec-capture/reference.csv
diff -u crates/damiao-codec/tests/fixtures/anvil_control.csv \
  /tmp/anvil-codec-capture/reference.csv
```

The container has no host devices or network. Replaying the committed CSV in
Rust and Python requires neither Docker nor Anvil.
