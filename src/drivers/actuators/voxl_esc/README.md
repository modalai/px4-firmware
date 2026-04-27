# VOXL ESC Driver — Spin-Up & Communication Health Logic

This document describes the motor spin-up monitoring, communication-timeout monitoring, and protection logic implemented in `voxl_esc.cpp` / `voxl_esc.hpp`. It is in addition to the regular ESC command/telemetry plumbing.

## Goal

Detect, as early and reliably as possible, when one or more motors fail to spin up — or when one or more ESCs stop communicating — and react safely:

1. Mark the affected motor(s)/ESC(s) in the `esc_status` uORB topic so PX4 commander logs and reports the fault.
2. Force-disarm the vehicle when a spin-up failure is detected (since PX4's built-in `fd_esc_arming_failure` failsafe is gated to a very short time window after arming and our timeout typically falls outside it).
3. While motors are still spinning up, hold all motor RPM commands at `rpm_min` so the controller does not push thrust higher until every motor is confirmed spinning.
4. After the spin-up phase, keep reporting motor stoppage and comms loss as faults but do **not** force-disarm by default — in-flight motor loss is a separate failure mode handled elsewhere.
5. If an ESC has not communicated since boot, keep it marked offline so commander's pre-arm checks block arming until every ESC is heard from.

## Phases

The driver tracks a single per-arm-cycle phase machine for each motor:

```
+----------+    arm     +-----------+   all motors confirmed   +-------------+
| Disarmed | ---------> | Spin-up   | -----------------------> | Post-spinup |
+----------+            +-----------+                          +-------------+
      ^                       |                                       |
      |                disarm |                                disarm |
      +-----------------------+---------------------------------------+
```

- **Disarmed** — vehicle is not armed. All tracking state (`_arm_time`, `_motors_spunup_mask`, `_spinup_first_seen[]`, `MOTOR_STUCK` failure bits, `_spinup_fail_disarm_sent`) is cleared. ESC comms timeout still runs: any ESC that has not reported recently keeps its bit cleared in `esc_online_flags`, which makes commander block arming.
- **Spin-up** — vehicle has just been armed; we are waiting for every motor to be observed in the `ESC_STATE_SPINNING` state continuously for at least `_spinup_min_duration_ms`. The RPM cap is active. The spin-up timeout is active. A comms loss in this phase is treated the same as a held-motor case: failure bit set, force-disarm published.
- **Post-spinup** — every motor has been confirmed spinning. The RPM cap is released. The spin-up timeout no longer fires. A motor stop is still reported as `FAILURE_MOTOR_STUCK` but does not force-disarm unless `_disarm_on_runtime_motor_stop` is set. A comms loss is logged as a warning only — no auto-disarm.

## Tracking State

All state is stored on the `VoxlEsc` instance (`voxl_esc.hpp`):

| Member                              | Type                            | Purpose                                                            |
|-------------------------------------|---------------------------------|--------------------------------------------------------------------|
| `_outputs_on`                       | `bool`                          | Mirror of `_mixing_output.armed().armed`. True ⇒ vehicle armed.    |
| `_prev_outputs_on`                  | `bool`                          | Used to detect arm/disarm transitions on each `Run()` iteration.   |
| `_arm_time`                         | `hrt_abstime`                   | Timestamp the vehicle was armed (0 when disarmed).                 |
| `_spinup_fail_disarm_sent`          | `bool`                          | Latches once we have published a force-disarm; prevents repeats.   |
| `_motors_spunup_mask`               | `uint8_t`                       | Bitmask of motors that have been confirmed spun up this arm cycle. |
| `_spinup_first_seen[N]`             | `hrt_abstime[N]`                | First timestamp this arm cycle that motor `id` was seen `SPINNING`.|
| `_spinup_min_duration_ms`           | `int32_t` (default `100`)       | Min continuous time motor must stay `SPINNING` to count.           |
| `_disarm_on_runtime_motor_stop`     | `bool` (default `false`)        | If true, force-disarm when a motor stops after spin-up.            |
| `_cap_rpm_during_spinup`            | `bool` (default `true`)         | If true, cap motor commands at `rpm_min` during spin-up phase.     |
| `_esc_comm_timeout_ms`              | `int32_t` (default `500`)       | An ESC with no telemetry for this many ms is treated as offline. `0` disables. |
| `_esc_offline_warning_logged[N]`    | `bool[N]`                       | Per-ESC latch so the offline warning logs only once per dropout.   |

There are also two relevant PX4 parameters:

| Parameter           | Default     | Purpose                                                                                  |
|---------------------|-------------|------------------------------------------------------------------------------------------|
| `VOXL_ESC_SPUP_TO`  | `3000` (ms) | Spin-up timeout: a motor that is not `SPINNING` past this many ms after arm is failed. `0` disables the check entirely. |
| `COM_ARM_CHK_ESCS`  | `0`         | Commander setting: when `1`, an offline ESC blocks arming pre-flight. **Set this to `1`** so that the offline-flag the driver maintains actually prevents the commander from arming. |

## Step by Step

### 1. Arm transition (`Run()`)

Inside the main `Run()` loop, immediately after copying the armed state from the mixer:

```cpp
_outputs_on = _mixing_output.armed().armed;

if (_outputs_on && !_prev_outputs_on) {           // just armed
    _arm_time = hrt_absolute_time();
    _spinup_fail_disarm_sent = false;
    _motors_spunup_mask = 0;
    for (int i = 0; i < VOXL_ESC_OUTPUT_CHANNELS; i++) {
        _esc_status.esc[i].failures &= ~(1u << esc_report_s::FAILURE_MOTOR_STUCK);
        _spinup_first_seen[i] = 0;
    }
} else if (!_outputs_on && _prev_outputs_on) {    // just disarmed
    _arm_time = 0;
}
_prev_outputs_on = _outputs_on;
```

The transition logic guarantees that every arm cycle starts from a clean slate: the spin-up clock starts now, no motor has been confirmed yet, no force-disarm has been sent yet, and any stale `MOTOR_STUCK` flag from the previous cycle is cleared.

### 2. Confirming a motor is spun up (`parse_response()`)

Each ESC sends back telemetry packets, each containing an `id_state` byte whose low nibble is one of:

- `ESC_STATE_NOT_SPINNING`
- `ESC_STATE_SPINNING_UP`
- `ESC_STATE_SPINUP_START_NO_CONTROL`
- `ESC_STATE_SPINNING`  ← the one we treat as "running"
- `ESC_STATE_UNKNOWN`

For each telemetry packet:

```cpp
if (state == esc_report_s::ESC_STATE_SPINNING) {
    if (_spinup_first_seen[id] == 0) {
        _spinup_first_seen[id] = tnow;                          // start timer
    }
    if ((tnow - _spinup_first_seen[id]) >= duration_us) {       // sustained?
        _motors_spunup_mask |= (1u << id);                       // confirm
    }
} else {
    _spinup_first_seen[id] = 0;                                  // reset
}
```

The `_spinup_first_seen[id]` field is the per-motor "first sighting in `SPINNING`" timestamp. If the motor flips back out of `SPINNING` before `_spinup_min_duration_ms` has elapsed, the timer resets. This filters out a single noisy telemetry packet from prematurely confirming a motor.

The phase-done test for the whole vehicle is then:

```cpp
spinup_phase_done = (_motors_spunup_mask == ((1u << VOXL_ESC_OUTPUT_CHANNELS) - 1));
```

### 3. Detecting a spin-up failure

Still in `parse_response()`, every time we get a telemetry packet from a motor, we evaluate the failure logic. There are two distinct cases.

**Case A — spin-up phase, motor not spinning past the timeout.**

```cpp
if (!spinup_phase_done && _parameters.esc_spinup_timeout_ms > 0) {
    hrt_abstime timeout_us = _parameters.esc_spinup_timeout_ms * 1000ULL;
    if ((hrt_absolute_time() - _arm_time) > timeout_us &&
        state != esc_report_s::ESC_STATE_SPINNING) {
        _esc_status.esc[id].failures |= (1u << esc_report_s::FAILURE_MOTOR_STUCK);

        if (!_spinup_fail_disarm_sent) {
            // publish VEHICLE_CMD_COMPONENT_ARM_DISARM with param2 = 21196 (force)
            _spinup_fail_disarm_sent = true;
        }
    }
}
```

**Case B — post-spin-up, a previously spun-up motor is no longer spinning.**

```cpp
if (spinup_phase_done && state != esc_report_s::ESC_STATE_SPINNING) {
    _esc_status.esc[id].failures |= (1u << esc_report_s::FAILURE_MOTOR_STUCK);

    if (_disarm_on_runtime_motor_stop && !_spinup_fail_disarm_sent) {
        // same force-disarm publish, gated by the runtime flag
    }
}
```

In Case B the failure bit is always set so commander still logs and reports the stall, but the force-disarm is gated by `_disarm_on_runtime_motor_stop` (currently `false`) so we do not aggressively kill a vehicle in flight when another failsafe should handle it.

### 4. Why we publish `vehicle_command` to force-disarm

PX4 commander already has a built-in `fd_esc_arming_failure` failsafe that maps to `Action::Disarm`. It is set automatically by `failure_detector` when `esc_status.esc[i].failures != 0`. However, `failsafe.cpp` only checks that flag inside this gate:

```cpp
if (time_us < _armed_time + COM_SPOOLUP_TIME * 1s) {
    CHECK_FAILSAFE(status_flags, fd_esc_arming_failure, Action::Disarm);
}
```

`COM_SPOOLUP_TIME` defaults to `1.0` seconds. Our `VOXL_ESC_SPUP_TO` timeout defaults to `3000` ms. By the time we set the failure bit, the failsafe gate has already closed, so the disarm never fires through that path.

To work around that, the driver publishes the equivalent of an RC stick disarm directly:

```cpp
vehicle_command_s vcmd{};
vcmd.command   = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
vcmd.param1    = 0.0f;     // 0 = disarm, 1 = arm
vcmd.param2    = 21196.f;  // magic value commander uses to force the action
vcmd.timestamp = hrt_absolute_time();
_vehicle_command_pub.publish(vcmd);
```

`21196.f` is the well-known force-disarm magic number used elsewhere in `Commander.cpp`. It tells commander to bypass the standard "in-flight, refuse to disarm" guard. The `_spinup_fail_disarm_sent` flag ensures we only publish once per arm cycle.

### 5. Capping RPM during spin-up

In `updateOutputs()`, once per call we compute `spinup_phase_done`:

```cpp
const uint8_t  all_spunup_mask = (1u << VOXL_ESC_OUTPUT_CHANNELS) - 1;
const bool     spinup_phase_done = (_motors_spunup_mask == all_spunup_mask);
```

For each output, after the existing per-channel max clamp, we additionally apply:

```cpp
if (_cap_rpm_during_spinup && !spinup_phase_done && !_turtle_mode_en
    && _parameters.rpm_min > 0 && outputs[i] > _parameters.rpm_min) {
    outputs[i] = _parameters.rpm_min;
}
```

So while any motor is still being verified, every motor receives at most a `rpm_min` setpoint regardless of what the mixer requested. This protects against a controller building up integrators / asking for thrust before the vehicle is mechanically ready.

The cap only applies in RPM mode, not PWM mode, and is disabled in turtle mode.

### 6. ESC communication timeout

In addition to the per-telemetry-packet logic above, the driver runs `check_for_esc_timeout()` once per `Run()` cycle. This is what catches the case where an ESC stops communicating entirely — `parse_response()` is never invoked for that ESC, so the spin-up tracking and post-spin-up tracking inside it cannot fire on their own.

For each ESC channel `i`:

```cpp
bool timed_out;
if (_esc_chans[i].feedback_time == 0) {
    timed_out = true;                                              // never heard from
} else {
    timed_out = (tnow - _esc_chans[i].feedback_time) > timeout_us; // stale
}
```

Whenever an ESC is timed out, its bit is cleared in both `esc_online_flags` and `esc_armed_flags`:

```cpp
_esc_status.esc_online_flags &= ~(1u << i);
_esc_status.esc_armed_flags  &= ~(1u << i);
```

Clearing `esc_online_flags` is what makes commander's `escCheck.cpp` report the ESC as offline and refuse to arm.

The action taken alongside that depends on the current phase:

**Disarmed.** Just clearing the flag is enough _provided commander is configured to require ESC checks_. The PX4 parameter `COM_ARM_CHK_ESCS` controls this and **defaults to `0` (disabled)** — set it to `1` for proper pre-arm blocking. With `COM_ARM_CHK_ESCS=0`, `escCheck.cpp` reports the offline ESC at `NavModes::None`, which logs the failure but does **not** clear `can_arm` bits, so commander still allows arming. With `COM_ARM_CHK_ESCS=1`, the same failure clears `can_arm` for `NavModes::All` and arming is blocked.

As a defence-in-depth fallback for the `COM_ARM_CHK_ESCS=0` case, the driver also force-disarms on the very first cycle after detecting an arm transition if any ESC is offline at that moment (see step 1 above). This guarantees the vehicle cannot stay armed for any meaningful duration even if commander's pre-arm checks let it through. The proper fix is still to set `COM_ARM_CHK_ESCS=1` so the arm command is rejected outright instead of accepted-and-immediately-undone.

The driver logs once per dropout via `_esc_offline_warning_logged[i]`. Because `_esc_status.esc_online_flags` is initialised to `0` in the constructor, every ESC starts out offline at boot and only becomes online after its first telemetry packet arrives — so the vehicle cannot be armed while any ESC is silent on startup.

**Armed, spin-up phase.** A comms loss in this phase is a real failure: we never get to confirm the motor is spinning. The driver mirrors what `parse_response()` would have done if it had been called — it sets `FAILURE_MOTOR_STUCK` on that ESC and publishes a force-disarm `vehicle_command` (gated by `_spinup_fail_disarm_sent` so it only fires once per arm cycle).

**Armed, post-spin-up.** A comms loss after the spin-up phase is logged as a warning (`PX4_WARN("ESC N comms lost in flight")`) and the bit stays cleared, but no force-disarm is published. In-flight loss of an ESC link is left to be handled by other layers (failsafe responses tied to attitude / motor failure detection / RC link, etc.).

The warning latches via `_esc_offline_warning_logged[i]` so a continuously-offline ESC does not spam the log. The latch is reset to `false` in `parse_response()` whenever a packet arrives from that ESC, so each new dropout will print one warning.

### 7. Setting `actuator_function`

When commander reports a faulty ESC (`"ESC {1}: motor stall"`), the `{1}` is the motor index, computed as:

```cpp
motor_index = esc_status.esc[i].actuator_function - ACTUATOR_FUNCTION_MOTOR1 + 1;
```

If `actuator_function == 0` (uninitialised), the calculation underflows for `uint8_t` and produces nonsense like "ESC 156". The driver therefore sets it explicitly each time it processes a telemetry packet:

```cpp
_esc_status.esc[id].actuator_function =
    actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + motor_idx;
```

`motor_idx` here is the user's PX4 motor mapping (`_output_map[id].number - 1`).

## Tuning Knobs

| Knob                              | Where                | What it controls                                                              |
|-----------------------------------|----------------------|-------------------------------------------------------------------------------|
| `VOXL_ESC_SPUP_TO` (parameter)    | `voxl_esc_params.c`  | Maximum allowed spin-up time per motor in ms. `0` disables the check.         |
| `_spinup_min_duration_ms`         | `voxl_esc.hpp`       | Required continuous-`SPINNING` duration to confirm a motor (default 100 ms). |
| `_disarm_on_runtime_motor_stop`   | `voxl_esc.hpp`       | If `true`, force-disarm when a motor stops after spin-up phase.               |
| `_cap_rpm_during_spinup`          | `voxl_esc.hpp`       | If `true`, hold all motors at `rpm_min` until the spin-up phase is done.      |
| `_esc_comm_timeout_ms`            | `voxl_esc.hpp`       | ms of silence before an ESC is treated as offline. `0` disables. (default 500 ms) |

## End-to-End Example

Successful arm of a healthy vehicle:

1. User commands arm.
2. `_outputs_on` flips to true → `_arm_time = now`, all per-motor state cleared.
3. Mixer requests motor outputs; `updateOutputs()` clamps each output to `rpm_min` because `_cap_rpm_during_spinup && !spinup_phase_done`.
4. ESCs spin up. Telemetry begins arriving with `state == SPINNING_UP`, then `SPINNING`.
5. For each motor, `_spinup_first_seen[id]` records the first `SPINNING` packet. After ≥ 100 ms of continuous `SPINNING`, that bit gets set in `_motors_spunup_mask`.
6. Once all four bits are set, `spinup_phase_done` becomes true. The RPM cap is dropped on the next `updateOutputs()` call. The spin-up failure check stops firing.
7. Vehicle flies normally.

Failed arm where one motor is held by hand:

1. User commands arm.
2. Same as above through step 3.
3. Three motors enter `SPINNING`. The held motor stays in `NOT_SPINNING`.
4. After 3 seconds, the held motor's branch in `parse_response()` sees `(now - _arm_time) > 3000 ms` and `state != SPINNING`. It sets `FAILURE_MOTOR_STUCK` on `esc_status.esc[id].failures`.
5. Driver publishes `VEHICLE_CMD_COMPONENT_ARM_DISARM` with the force magic number.
6. Commander disarms; vehicle is safe.
7. `_spinup_fail_disarm_sent = true` prevents repeated disarm commands.

ESC unplugged before arming (with `COM_ARM_CHK_ESCS=1`, recommended):

1. Vehicle is powered on. `esc_status.esc_online_flags` starts at `0`.
2. Three healthy ESCs report telemetry; their bits in `esc_online_flags` are set in `parse_response()`. The fourth (unplugged) ESC never reports.
3. `check_for_esc_timeout()` runs every `Run()` cycle. It sees `_esc_chans[3].feedback_time == 0` and keeps bit 3 cleared. It logs `"ESC 4 offline (pre-arm) — arming will be blocked"` once.
4. User attempts to arm. Commander's `escCheck.cpp` notices `online_bitmask != esc_status.esc_online_flags`, emits `"ESC 4 offline"`, and refuses to arm because `required_modes == NavModes::All`.
5. User reconnects the ESC. Its first telemetry packet arrives. `parse_response()` sets bit 3 in `esc_online_flags` and clears the warning latch. Commander's pre-arm check now passes and arming succeeds.

ESC unplugged before arming (with `COM_ARM_CHK_ESCS=0`, default — fallback path):

1. Same boot sequence as above; ESC 4 stays offline.
2. User commands arm. Commander allows it (because `required_modes == NavModes::None`, the offline flag is informational only).
3. `_outputs_on` flips to true. The arm-transition block in `Run()` checks `esc_online_flags & expected_online_mask` and finds bit 3 still clear.
4. Driver immediately publishes a force-disarm `vehicle_command` and sets `FAILURE_MOTOR_STUCK` on ESC 4. The vehicle disarms within one `Run()` cycle. Log: `"Arm rejected: one or more ESCs offline (online_flags=0x07) — forcing disarm"`.

The fallback path is correct but ugly — the vehicle is briefly armed before being told to disarm. Always prefer setting `COM_ARM_CHK_ESCS=1`.

ESC link drops in flight:

1. Vehicle is armed and flying; `_motors_spunup_mask == 0x0F`, so `spinup_phase_done` is true.
2. ESC 2's signal cable comes loose. Telemetry stops.
3. `check_for_esc_timeout()` sees `(now - _esc_chans[1].feedback_time) > 500 ms`, clears bit 1 in `esc_online_flags`, and logs `"ESC 2 comms lost in flight"` once.
4. Because we are post-spin-up, no force-disarm is published. The vehicle keeps flying; commander still surfaces the offline ESC in its health report. Reaction to the comms loss is left to other failsafes (e.g. attitude/motor-failure detector or pilot intervention).
