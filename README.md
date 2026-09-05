# ros2_resilience

A closed-loop resilience regression harness for ROS 2. It answers one
specific question, reproducibly: **when a communication fault is injected
into a running ROS 2 graph, does the system detect it, recover, and reach a
safe state within explicit deadlines** — with every step backed by
independent evidence, not a single component's self-report.

```
resilience contract (YAML)
        │
        ▼
 experiment runner ──activates──▶ fault injector ──▶ watchdog ──▶ recovery command
        │                              │                │              │
        │                         faulted /odom     violation      /cmd_vel
        ▼                              │                │              │
   observed evidence ◀─────────────────┴────────────────┴──────────────┘
        │
        ▼
 behavioral assertions → measured latencies → PASS / FAIL (JSON)
```

Four fault modes are covered end to end: **dropout**, **latency**, **stale
hold**, and a **healthy control** (no fault, proving the harness doesn't cry
wolf). Every one of them runs against a real ROS 2 Jazzy graph — real nodes,
real topics, real DDS message passing — not a simulation of the pipeline.

## Why this exists

ROS 2 already has fault-injection and fuzzing tools. This project's
contribution is narrower and more specific: a **closed-loop regression
experiment**. It doesn't just inject a fault — it defines, in one YAML file,
exactly what "the system correctly handled it" means (which topic must show
degradation, which watchdog reason must fire, how fast recovery must be
published *and separately observed*, how long the robot must stay stopped),
then runs the real system and produces a versioned, machine-checkable PASS
or FAIL with the measured evidence attached.

## Architecture

The package is split so that the interesting logic is pure, deterministic,
and fast to test — ROS is an adapter around it, not a dependency of it.

```
ros2_resilience/
├── core/           pure Python: scenario config, YAML validation, the
│                   CREATED→...→FINISHED lifecycle state machine, normalized
│                   events, behavioral assertions, metrics, JSON results,
│                   batch/seed bookkeeping. No rclpy import anywhere.
├── faults/         pure fault injection: dropout, latency (with jitter),
│                   stale-hold, and the injector that coordinates exactly
│                   one active mode at a time.
├── robot/          pure planar kinematic robot model.
├── health/         pure watchdog: timestamp-age + trailing-window
│                   loss-fraction detection, latched recovery.
└── ros/            the only place rclpy is imported: robot/injector/
                    watchdog nodes, the experiment-runner orchestrator,
                    and the batch CLI.
```

Every fault mode's *effect* is confirmed by a stream the fault itself
doesn't control: dropout is confirmed by advancing raw traffic plus a
deficit on the faulted stream; latency by matching a raw/faulted pair on
their shared source timestamp and measuring the delivery gap; stale hold by
a frozen faulted-stream timestamp while the raw stream keeps advancing. The
watchdog's own claim of publishing a recovery command is checked against
the robot's independently observed `/cmd_vel` traffic — no component
grades its own homework.

## What's a deliberate Phase-1 simplification

Being upfront about scope, since a resilience-testing tool that oversells
its own guarantees would be a bad look:

- **Trial isolation is graph-level, not process-level.** Each trial gets a
  fresh `rclpy.Context`, fresh nodes, and namespaced topics, but all trials
  in a batch run sequentially in one OS process rather than fresh processes
  per trial. The design this project started from called for full process
  isolation; that was judged out of scope for the time available.
- **Fault (re)configuration is a direct, typed method call**
  (`FaultInjectorNode.apply_config`) from the in-process experiment runner,
  not a ROS parameter service call over the wire. Since every node in a
  trial lives in the same process, this is the real equivalent of what a
  parameter service would do across processes — without the async-client
  complexity a purely local call doesn't need.
- **Batches run in real wall-clock time.** Nothing here is time-accelerated,
  so a scenario with a 4-second observation window takes about 4 real
  seconds per trial.
- **One host, `use_sim_time=false`.** Cross-process latency measurements use
  ROS clock timestamps; scheduling deadlines use monotonic time. This is
  correct on one host without simulated time; it is not a claim about
  multi-host or simulated-time correctness.

## Repository layout

```
ros2_resilience/          the ROS 2 Python package (ament_python)
launch/                   a manual, interactive launcher (fault disabled by
                           default — see its docstring for why)
scenarios/                four ready-to-run resilience-contract YAMLs
tests/unit/                184 pure-Python tests, no ROS required
tests/integration/         real ROS 2 tests: full trials end to end, plus a
                           launch_testing test that launches robot/injector/
                           watchdog as three separate OS processes
```

## Quickstart

Everything here targets ROS 2 Jazzy on Ubuntu 24.04. The stock
`ros:jazzy-ros-base-noble` image already contains everything the build and
tests need (colcon, pytest, launch_testing), so on macOS or any host
without a native ROS install:

```bash
docker run --rm -it \
  --mount type=bind,source="$PWD",target=/workspace/src/ros2_resilience \
  --workdir /workspace \
  ros:jazzy-ros-base-noble bash
```

Inside the container (or any sourced ROS 2 Jazzy workspace):

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace && colcon build --symlink-install --packages-select ros2_resilience
source install/setup.bash
cd src/ros2_resilience

# 184 pure-Python tests, no ROS graph needed, ~0.1s
python3 -m pytest tests/unit -q

# One real trial of each shipped scenario, against real ROS 2 nodes
python3 -m pytest tests/integration/test_scenarios_end_to_end.py -q

# Three real OS processes, launched and observed over real topics
launch_test tests/integration/test_launch_multiprocess.py

# Run a scenario for real and get a versioned JSON result
ros2 run ros2_resilience run_scenario \
  --scenario scenarios/dropout_burst.yaml \
  --repeat 5 \
  --output results/dropout_burst.json
```

`run_scenario` exits `0` when every requested trial passes, `1` on a
behavioral regression, and `2` on a configuration or infrastructure error —
so it composes directly into CI as a pass/fail gate.

## Example: a real measured result

This is actual output from `dropout_burst.yaml` (80% drop probability, real
watchdog, real recovery), not a hand-written example:

| Assertion | Result | Measured |
|---|---|---|
| No violation before activation | ✅ | — |
| Fault independently observable | ✅ | raw traffic advanced, injector dropped 17 of 20, faulted-stream deficit confirmed |
| Watchdog detected `loss_fraction_exceeded` | ✅ | 0.552 s (≤ 1.5 s) |
| Recovery command published | ✅ | 0.0013 s (≤ 0.1 s) |
| Recovery command *observed* on `/cmd_vel` | ✅ | 0.0070 s (≤ 0.3 s) |
| Robot reached stopped state | ✅ | 0.597 s (≤ 2.0 s) |
| Stopped state held | ✅ | for the full 0.25 s |

A mutation test proves this isn't rubber-stamping: suppressing the
watchdog's actual `/cmd_vel` publish call (while leaving its self-reported
"I published" diagnostic intact) correctly fails `recovery_command_observed`
and `robot_stopped` — because those two assertions are graded from the
robot's own independent observation, not the watchdog's word for it.

## Resilience-contract YAML

```yaml
schema_version: 1
name: dropout_burst
system:
  input_topic: /odom_raw
  output_topic: /odom
  command_topic: /cmd_vel
  robot: {publish_rate_hz: 20.0, initial_linear_velocity_mps: 0.5, initial_angular_velocity_rad_s: 0.0}
experiment:
  seed: 11
  readiness_timeout_sec: 10.0
  healthy_baseline_sec: 2.0
  observation_after_activation_sec: 4.0
fault:
  type: dropout
  parameters: {drop_probability: 0.8}
  duration_sec: 3.0
watchdog:
  max_age_sec: 1.0
  expected_rate_hz: 20.0
  loss_window_sec: 1.0
  max_loss_fraction: 0.5
  check_period_sec: 0.01
  recovery: stop
expect:
  no_violation_before_fault: true
  fault: {observable: true, within_sec: 1.0}
  watchdog: {violation: loss_fraction_exceeded, within_sec: 1.5}
  recovery: {action: stop, command_within_sec: 0.1, observed_within_sec: 0.3}
  robot: {linear_velocity_below_mps: 0.01, angular_velocity_below_rad_s: 0.01, within_sec: 2.0, hold_sec: 0.25}
```

Unknown fields, unsupported fault/recovery types, out-of-range values, and
deadlines that don't fit inside the observation window are all rejected at
load time — see `ros2_resilience/core/validation.py` and
`tests/unit/test_validation.py`.

## License

Apache-2.0. See [LICENSE](LICENSE).
