# External Flight Mode Transport and Code-Sharing Analysis

## Scope

This document compares the VOXL2 `flight_mode_bridge` with PX4's ROS 2
external-mode path over uXRCE-DDS and Zenoh. It identifies which behavior is
already shared, which code could reasonably be shared, and which transport
concerns should remain separate.

For the implementation-quality and safety review of the bridge itself, see
[Flight Mode Bridge Code Review](CODE_REVIEW.md).

## Conclusion

The new bridge is conceptually doing the same job as the ROS 2 external-mode
stack at the PX4 contract boundary: it registers an external component,
participates in arming checks, configures a setpoint type, publishes setpoints,
and reports mode activation state.

It is not equivalent to either PX4 middleware module internally. The
uXRCE-DDS and Zenoh modules are generic transports for selected uORB topics.
They do not implement the external-mode lifecycle. With ROS 2, most of that
lifecycle is implemented on the companion computer by the
[`px4-ros2-interface-lib`](https://github.com/Auterion/px4-ros2-interface-lib).
The MPA bridge combines transport adaptation with a proxy implementation of
that lifecycle inside PX4.

```text
ROS 2 mode + px4-ros2-interface-lib
        |
        +-- uXRCE-DDS or Zenoh -------------------+
                                                   |
MPA mode app                                       v
        |                              standard external-mode uORB
        +-- px4_modes --> flight_mode_bridge ----> Commander / ModeManagement
```

The correct shared boundary is therefore the standard external-mode uORB
contract and Commander, not a common DDS, Zenoh, and MPA transport loop.

## Responsibilities of Each Path

PX4's [uXRCE-DDS topic map](../../../../../../src/modules/uxrce_dds_client/dds_topics.yaml)
and [Zenoh topic map](../../../../../../src/modules/zenoh/dds_topics.yaml) both
transport the external-mode topics, including:

- `register_ext_component_request` and `register_ext_component_reply`;
- `unregister_ext_component`;
- `arming_check_request` and `arming_check_reply`;
- `setpoint_config` and `setpoint_config_reply`;
- `trajectory_setpoint` and `vehicle_attitude_setpoint`; and
- vehicle telemetry such as `vehicle_attitude`, `vehicle_local_position`, and
  `vehicle_status`.

The transports differ mainly in discovery, serialization, link management,
and quality-of-service configuration. PX4's middleware documentation describes
[uXRCE-DDS](../../../../../../docs/en/middleware/uxrce_dds.md) and
[Zenoh](../../../../../../docs/en/middleware/zenoh.md) as mechanisms for making
uORB data available to ROS 2 applications.

The application-facing behavior differs as follows:

| Operation | ROS 2 over DDS or Zenoh | MPA `flight_mode_bridge` |
| --- | --- | --- |
| Registration | The application publishes the standard PX4 request. | The application sends `MODE_MSG_REGISTER_REQ`; the bridge publishes the PX4 request. |
| Registration reply | The application receives the PX4 reply directly. | The bridge correlates the PX4 reply and emits `MODE_MSG_REGISTER_REPLY`. |
| Arming checks | The application receives each poll and answers it. | The application streams health state; the bridge caches it and answers PX4 polls. |
| Setpoint configuration | The application exchanges native `SetpointConfig` messages. | The bridge derives and publishes the configuration from the MPA registration. |
| Setpoints | The application publishes native PX4 setpoint messages. | The application sends packed MAVLink setpoints; the bridge validates and converts them. |
| Telemetry | Applications subscribe to individual uORB-derived topics. | The bridge combines selected topics into `MODE_MSG_VEHICLE_STATE`. |
| Multiple applications | Middleware provides publishers, subscribers, and identities. | One MPA pipe is multiplexed using a random 64-bit request ID. |

The [PX4 ROS 2 Control Interface documentation](../../../../../../docs/en/ros2/px4_ros2_control_interface.md)
describes the higher-level mode, executor, health, requirements, and setpoint
abstractions provided by the companion-side library. Those abstractions, not
the middleware transport modules, are the closest conceptual match for the
MPA bridge.

## What Is Already Shared

All three external paths converge on the same PX4 implementation:

- `ModeManagement` allocates the EXTERNAL1 through EXTERNAL8 navigation-state
  slots and processes registration and setpoint configuration;
- `ExternalChecks` polls registered external components and incorporates their
  health and mode requirements into arming and failsafe decisions;
- the versioned uORB external-mode messages define the PX4-facing protocol;
- PX4 controllers consume the same native setpoint topics regardless of the
  original transport; and
- `vehicle_status.nav_state` represents activation in the same way for all
  providers.

This is useful existing code sharing: PX4 has one source of truth for mode
allocation, control configuration, health integration, and navigation state.
The MPA bridge should remain an adapter to this source of truth rather than
creating an independent external-mode subsystem.

## Recommended Code-Sharing Opportunities

### 1. Extract a reusable external-mode session helper

The bridge currently implements the external-mode conversation directly in a
large module class. A transport-neutral `ExternalModeSession`-style helper
could own:

- request/reply correlation during registration;
- assigned mode and arming-check IDs;
- setpoint configuration and its result;
- cached arming health and mode requirements;
- activation and deactivation tracking;
- stale-client retirement; and
- deterministic unregistration and shutdown cleanup.

The bridge would own up to eight session objects and retain responsibility for
MPA parsing, pipe I/O, and routing by request ID. In-firmware custom modes such
as `mc_nn_control` and `mc_raptor` could potentially use one session object
instead of independently implementing the uORB registration lifecycle.

This helper would not normally be called by the uXRCE-DDS or Zenoh modules.
Those modules do not own a mode session; they serialize and transport messages
for a companion-side application that owns it.

### 2. Share MAVLink setpoint validation and conversion

The MPA protocol deliberately transports MAVLink
`SET_POSITION_TARGET_LOCAL_NED` and `SET_ATTITUDE_TARGET` payloads. The bridge
then duplicates parts of the translation already performed by PX4's
[`MavlinkReceiver`](../../../../../../src/modules/mavlink/mavlink_receiver.cpp).

Pure helper functions could be extracted from the MAVLink receiver to:

- validate coordinate frames and ignore masks;
- require at least one valid position, velocity, or acceleration field;
- convert ignored values consistently to `NAN`;
- validate attitude quaternions and thrust values; and
- select the correct scalar-thrust body axis for multicopter, fixed-wing, and
  VTOL operation.

Both the normal MAVLink receiver and `flight_mode_bridge` could call these
helpers. This is the highest-value sharing opportunity inside the PX4 tree
because it removes duplicated flight-control interpretation and prevents the
two entry points from drifting apart.

The helpers should accept decoded payload structures and return native PX4
setpoints plus an explicit result code. They should not depend on MPA pipe
state or MAVLink channel state, which would make isolated unit tests possible.

### 3. Establish one source of truth for the MPA wire protocol

`voxl_px4_modes_pipe.h` currently says that its original lives in an
application repository and that the PX4 copy must be synchronized manually.
That creates a compatibility risk independent of the external-mode logic.

A shared ModalAI protocol package, or generated protocol artifacts, should
provide:

- message IDs and packed layouts;
- protocol version constants;
- maximum-payload calculations and static assertions;
- encode/decode validation; and
- compatibility tests using known byte sequences.

PX4 and every MPA client would then consume the same release of the protocol.
This is likely the most practical cross-repository code-sharing improvement.

### 4. Match the ROS 2 library's lifecycle semantics

Directly linking `px4-ros2-interface-lib` into an MPA application is unlikely
to be useful because it is built around ROS 2 nodes, `px4_msgs`, discovery,
topics, and QoS. Its behavior is nevertheless a valuable reference.

The MPA implementation should provide equivalent semantics where the PX4
contract is the same, particularly:

- external API and message compatibility checks;
- registration failure reporting;
- setpoint-type support results;
- derived estimator requirements;
- structured health or arming-failure events;
- client liveness and cleanup; and
- clear activation-state transitions.

One concrete gap is `SetpointConfigReply`. Its derived requirement flags are
intended to be applied to subsequent `ArmingCheckReply` messages; PX4 does not
apply them automatically. The bridge currently logs the reply rather than
incorporating or forwarding those requirements. A reusable session helper
should own this behavior.

### 5. Consider a portable companion-side core only if it has multiple users

If ModalAI expects to maintain both ROS 2 and non-ROS implementations of
several external modes, a small transport-independent companion library could
define the application lifecycle:

```text
ExternalModeClient
    +-- Ros2Transport  --> px4_msgs topics
    +-- MpaTransport   --> px4_modes messages
```

The portable layer could expose registration, activation callbacks, health,
and typed setpoint APIs. The adapters would handle the different transports.

This is a larger cross-repository design effort and should be driven by actual
shared applications. For a single MPA example, sharing the wire protocol and
PX4-side conversion logic provides more value with less coupling.

## Code That Should Remain Separate

The following responsibilities have materially different constraints and
should not be forced behind one implementation:

- MPA server creation, callbacks, client multiplexing, and packed-message I/O;
- XRCE session and DDS entity management;
- Zenoh sessions, key expressions, and per-topic transport options;
- ROS 2 discovery and QoS handling; and
- the MPA-specific aggregated vehicle-state packet.

Similarly, turning the MPA bridge into a general-purpose uORB transport would
largely recreate capabilities already provided by uXRCE-DDS and Zenoh. Its
useful distinction is a small, dependency-light API designed specifically for
external modes.

## Proposed Architecture

```text
boards/modalai/voxl2/.../flight_mode_bridge
    MPA parsing, routing, telemetry aggregation, pipe I/O
                         |
                         v
ExternalModeSession helper
    registration, IDs, health, requirements, activation, cleanup
                         |
                         v
Standard versioned external-mode uORB messages
                         |
                         v
Commander ModeManagement and ExternalChecks

MAVLink receiver -----------------------------+
                                               |
flight_mode_bridge ----------------------------+--> shared setpoint conversion
```

The exact home of an `ExternalModeSession` helper depends on its users. If it
is used by generic PX4 modules, it belongs under `src/lib`. If it remains
VOXL2-specific, a board-local library avoids presenting it as a supported PX4
API prematurely. MAVLink setpoint conversion is generic and should live near
the existing MAVLink or mode setpoint utilities.

## Recommended Implementation Order

1. Address the bridge's existing lifecycle and safety findings, especially
   stale-client cleanup and the unimplemented setpoint-timeout guarantee.
2. Extract and test the shared MAVLink setpoint conversion and validation.
3. Separate MPA transport handling from an external-mode session object.
4. Move the MPA protocol definitions to a shared or generated package.
5. Add semantic parity for configuration replies, mode requirements, health
   events, and compatibility checks.
6. Consider a transport-independent companion library only after identifying
   at least two applications that would consume it.

This sequence preserves the current architecture while concentrating shared
code in the areas that affect correctness and compatibility.
