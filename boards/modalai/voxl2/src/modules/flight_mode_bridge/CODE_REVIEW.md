# Flight Mode Bridge Code Review

## Review scope

This document reviews commit `7487867e37ddfd11aef83c1dee4223273f9d597c`,
which adds the VOXL2 `flight_mode_bridge` module and the `px4_modes` MPA wire
protocol.

The review compares the new code against:

- the repository's [contribution requirements](../../../../../../CONTRIBUTING.md),
  AStyle configuration, and the
  [PX4 coding guidelines](https://docs.px4.io/main/en/contribute/code);
- PX4's existing external-mode, setpoint, and MAVLink implementations; and
- neighboring VOXL2 MPA modules, particularly `crsf_bridge` and
  `vehicle_local_position_bridge`.

This was a source-level review. `git diff --check` and an AStyle dry run were
used, but the code was not built, linked, or executed.

## Executive summary

The commit is mechanically well formatted and fits the existing VOXL2 module
architecture. It uses the expected `ModuleBase`/work-queue pattern, integrates
with the board-local MPA library, uses modern uORB wrappers, and contains useful
protocol documentation.

It is not ready for upstream review or flight use without additional work. The
most important concerns are that the advertised 500 ms setpoint-loss failsafe
is not implemented by this PX4 tree, crashed applications are not reclaimed,
and broadcast estimator data loses its validity and sample-age information.
There are also missing license headers, no tests, and several smaller PX4 style
and defensive-programming issues.

## Findings

### High: the advertised 500 ms setpoint timeout is not active

The bridge sets `setpoint_config_s::timeout_ms` to 500 ms in
[`publish_setpoint_config()`](flight_mode_bridge.cpp#L141-L150), and both the
module and protocol comments describe this as arming a setpoint-loss failsafe.

In this tree,
[`ModeManagement::checkConfigControlSetpointUpdates()`](../../../../../../src/modules/commander/ModeManagement.cpp#L609-L660)
uses the setpoint type, source ID, and `should_apply`, but never reads
`timeout_ms`. The commit that originally introduced `SetpointConfig` also
states that the timeout field was added for a future extension and was not yet
implemented.

As a result, a client can continue sending health messages while its setpoint
loop has stopped, and the promised 500 ms protection will not be provided by
`SetpointConfig`. The comments currently overstate the safety behavior.

Recommended action: either implement the PX4 setpoint timeout end to end or
remove the claim and add a bridge-side setpoint watchdog that is independent
of the general client-aliveness timestamp.

### High: crashed clients are not reclaimed

The bridge records one client for each request ID and clears it only after an
explicit `MODE_MSG_UNREGISTER`. A stale client is still answered during every
PX4 arming-check poll; only `can_arm_and_run` is changed to false
([arming-check handling](flight_mode_bridge.cpp#L206-L229)).

PX4's
[`ExternalChecks::update()`](../../../../../../src/modules/commander/HealthAndArmingChecks/checks/externalChecks.cpp#L223-L310)
counts a timely reply as proof that the registered component is responsive,
regardless of `can_arm_and_run`. Because the bridge keeps replying on behalf of
the dead client, Commander does not classify that registration as unresponsive
and does not reclaim it.

A restarted application normally has a new random request ID, so it allocates
another bridge and Commander slot. Repeated crashes can exhaust all eight slots
while the bridge remains running. The stale entry also keeps the 50 Hz vehicle
state broadcast enabled.

The module's `cleanup()` closes the MPA pipe without explicitly unregistering
its current clients. Commander can eventually detect the whole bridge as
unresponsive after the bridge stops, but explicit cleanup would be faster and
more deterministic.

Recommended action: track a stale-client retirement timeout and unregister
inactive clients after a grace period. Also unregister all valid clients during
module shutdown. Re-registration by name could reclaim a stale matching entry.

### Medium-high: vehicle state discards estimator validity and sample age

The vehicle-state broadcast ignores the return value from the local-position
and attitude subscriptions, drops all position and velocity validity flags,
and stamps the combined packet with the current bridge time
([broadcast construction](flight_mode_bridge.cpp#L281-L316)). The activation
snapshot has the same local-position issue.

If a subscription copy fails, the zero-initialized output looks like a vehicle
at the origin with an invalid all-zero quaternion. Clients also cannot tell
whether individual position, velocity, and attitude samples are stale.

This is weaker than the neighboring
[`vehicle_local_position_bridge`](../vehicle_local_position_bridge/vehicle_local_position_bridge.cpp#L113-L180),
which checks PX4 validity flags and writes `NAN` for unavailable values.

Recommended action: extend the protocol with source timestamps and explicit
position, velocity, attitude, and heading validity. At minimum, check each copy
and use `NAN` for unavailable floating-point fields.

### Medium: scalar attitude thrust is multicopter-specific

When `THRUST_BODY_SET` is absent, the bridge always maps scalar thrust to body
`-Z` ([attitude translation](flight_mode_bridge.cpp#L509-L537)). That is the
multicopter convention.

PX4's normal
[`MavlinkReceiver::fill_thrust()`](../../../../../../src/modules/mavlink/mavlink_receiver.cpp#L1743-L1785)
maps scalar thrust to body `+X` for fixed-wing and selects the appropriate axis
for the current VTOL state. The bridge neither performs that selection nor
rejects non-multicopter vehicles.

Recommended action: either document and enforce a multicopter-only restriction
or use PX4-equivalent vehicle-type-aware thrust mapping.

### Medium: trajectory validation does not fully mirror MAVLink handling

The bridge publishes `trajectory_setpoint_s` even when position, velocity, and
acceleration are all masked out
([trajectory translation](flight_mode_bridge.cpp#L442-L488)). PX4's regular
MAVLink receiver rejects that case as an invalid setpoint
([MAVLink validation](../../../../../../src/modules/mavlink/mavlink_receiver.cpp#L1262-L1293)).

This contradicts the comments stating that the translation is identical to or
mirrors `mavlink_receiver.cpp`.

Recommended action: factor the shared validity rules into a small testable
translation helper and reject setpoints without any position, velocity, or
acceleration component.

### Medium: registration and wire-input validation is incomplete

The registration path should validate more of the message before publishing to
Commander:

- `mode_register_req_t::name` is printed with `%s` without first forcing NUL
  termination ([registration handling](flight_mode_bridge.cpp#L350-L421)). A
  malformed client can cause an out-of-bounds read while logging.
- Empty names and unsupported registration-flag combinations are accepted.
- A request with `register_mode == 0` can produce a successful reply with
  `mode_id == -1`; the bridge then casts that ID to `uint8_t` and publishes a
  setpoint configuration for source ID 255.
- Malformed headers, partial trailing messages, unknown message types, and
  undersized payloads are silently discarded, making integration failures
  difficult to diagnose.

Recommended action: sanitize the mode name immediately, require the flags that
the bridge's one-mode-per-client contract needs, validate returned IDs before
using them, and add rate-limited diagnostics for malformed messages.

### Medium-low: pipe writes and payload bounds are not checked

[`pipe_send()`](flight_mode_bridge.cpp#L579-L586) returns the result of
`MPA::PipeWrite()`, but every caller ignores it. Registration replies and
activation messages can therefore be lost without a diagnostic.

The neighboring
[`crsf_bridge`](../crsf_bridge/crsf_bridge.cpp#L145-L203) checks write failures
and reports malformed control packets. It is a better local model for error
handling.

The fixed 128-byte payload buffer is sufficient for the current messages, but
there is no compile-time assertion or runtime check tying it to the largest
protocol payload. A future protocol extension could turn that into a stack
overflow.

Recommended action: define a protocol maximum payload constant, statically
assert every outbound payload against it, reject oversized sends, and report
write failures with rate limiting.

### Low: mutex scope and lifecycle can be made safer

The code manually locks and unlocks a POSIX mutex around large sections that
include uORB work and MPA writes. The current paths appear balanced, but future
early returns would be error-prone and a blocking pipe write delays incoming
setpoint processing.

Recommended action: use PX4's `LockGuard`, minimize the protected region, avoid
holding the registry lock during external I/O where possible, and destroy the
mutex during final cleanup.

## PX4 coding-standard assessment

### What matches well

- All three new C/C++ files pass the repository AStyle configuration unchanged.
- `git diff --check` reports no whitespace errors.
- Indentation, braces, `.cpp` naming, private-member prefixes, and initialization
  generally match PX4 conventions.
- Time constants include units in their names.
- Magic values and protocol fields are documented.
- The vendored MAVLink structures have useful compile-time layout assertions.
- Modern `uORB::Publication` and `uORB::Subscription` wrappers are used.

### What does not match

- The four newly created files have no BSD 3-clause copyright/license header.
  This differs from PX4's contribution requirement and every neighboring
  VOXL2 module and CMake file.
- Helper methods such as `publish_setpoint_config`, `handle_msg`, `pipe_send`,
  and `find_client` use snake case instead of the current PX4 lower-camel-case
  convention. Framework-required names such as `task_spawn` are established
  exceptions in this tree.
- Several C-style casts should be `static_cast`.
- The `ModuleBase::Descriptor` definition is 141 characters, one character
  over PX4's documented maximum.
- `print_usage()` lacks `PRINT_MODULE_DESCRIPTION` and categorizes the module
  as `"command"`; neighboring VOXL2 bridges use `"system"`.
- The 675-line implementation combines transport parsing, client lifecycle,
  MAVLink translation, uORB interaction, and module lifecycle in one file,
  which makes isolated testing difficult.
- The commit adds no unit or integration tests. The parser, registration state
  machine, timeout behavior, and setpoint-mask translation are all practical
  unit-test candidates.
- The commit message does not use the repository's current
  `type(scope): description` convention, has no `Signed-off-by` trailer, and
  contains the typo `offboarb`.

The AStyle result demonstrates good mechanical formatting, but it does not
cover the licensing, naming, design, or safety issues above.

## VOXL2 custom-code assessment

### What matches well

- The module is located with the other VOXL2 board-specific modules.
- Its CMake dependency and include structure match other MPA bridges.
- It follows the local `ModuleBase` plus work-queue lifecycle pattern.
- MPA initialization, server creation, and control-callback registration are
  familiar local patterns.
- Publishing uORB data from the MPA callback follows the `crsf_bridge`
  precedent.
- Raw packed messages, a magic value, and direct MPA pipe writes are consistent
  with existing VOXL transport code.
- Build inclusion and automatic startup follow the neighboring bridge modules.
- Resource cleanup is more complete than in some of the older VOXL2 bridges.

### Where it differs

- It lacks the license headers used by all neighboring modules.
- It has less input and output error reporting than `crsf_bridge`.
- It handles estimator validity less carefully than
  `vehicle_local_position_bridge`.
- Its module help is less complete than the neighboring bridges.
- The wire header is maintained as a manual copy whose declared source of truth
  is another repository. Existing VOXL bridges generally consume protocol
  structures from a shared installed header instead.
- It has substantially more state and flight-safety responsibility than the
  other bridges, but no corresponding tests, status counters, or recovery
  behavior.

## Recommended order of work

1. Correct the setpoint-timeout behavior or remove the false 500 ms guarantee.
2. Add stale-client reclamation and deterministic shutdown unregistration.
3. Preserve estimator validity and source timestamps in the wire protocol.
4. Harden registration, message, returned-ID, and setpoint validation.
5. Restrict attitude operation to multicopters or implement vehicle-aware
   thrust mapping.
6. Check MPA write results and make the maximum payload size explicit.
7. Add BSD license headers and complete module usage documentation.
8. Split protocol translation and lifecycle logic into testable components.
9. Add parser, lifecycle, stale-client, setpoint-mask, and timeout tests, then
   perform the required Docker build and target/SITL or bench validation.

## Overall assessment

- Mechanical PX4 formatting: **good**
- PX4 naming and documentation conventions: **mixed**
- PX4 contribution readiness: **not ready**
- VOXL2 architectural consistency: **good**
- Defensive robustness and flight-safety confidence: **needs significant work**
