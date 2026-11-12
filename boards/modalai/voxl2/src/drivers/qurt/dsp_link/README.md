# DSP link

`dsp_link` is the PX4 driver that receives CDSP messages through the ADSP
`dsp_link` API, validates them, and processes their contents. Its first
supported payload is odometry, which it publishes as
`vehicle_visual_odometry` on SLPI. `voxl-px4-start` starts the driver
automatically on M0197 (QCS6490).

The receive callback maintains reception statistics and passes the current
packet to `process_odometry()` for validation and publication. Future message
types belong in separate handlers selected by the receive path. The current
backend still delivers one complete odometry packet per callback; typed
messages and queue delivery require a coordinated producer/transport protocol
change.

The SLPI library requires an ADSP image exporting the `dsp_link` API. Rebuild
and deploy `modalai-adsp` with its automatic test receiver removed before using
this module: the link permits only one receiver. Keep the CDSP sender running.
Use the normal VOXL2 Docker build and deployment scripts for this checkout;
deploy the updated `voxl-px4-start` as well as the PX4 binaries.

From the PX4 shell:

```sh
qshell dsp_link status
qshell dsp_link stop
qshell dsp_link start
qshell listener vehicle_visual_odometry 5
```

From a target Linux shell, use `px4-qshell` in place of `qshell`.

## Status

Status reports whether the module is stopped, waiting for data, receiving
without publishing, publishing, or stale. A two-second silence is marked stale
for diagnostics; it does not discard the next sample. Counters are cumulative
since the last new start attempt and remain available after stopping.

The output includes:

- Received and published counts, sequence gaps, and producer restarts.
- Rejections grouped by length, envelope, sequence, timestamp, filter/camera
  state, coordinate frame, numeric values, and uORB publication errors.
- Mean receive rate, most recent transmit/receive intervals, and time since
  the last callback and publication.
- Minimum, mean, maximum, and latest sample-to-callback age in microseconds.
- Latest producer generation, sequence, filter generation, session, camera
  state, frame identifiers, quality, reset counter, and position.
- The most recent API error and the last packet's rejection reason.

The existing CDSP automatic test sender sets `source_session` to `TEST`,
filter state and camera count to zero, unknown coordinate frames, and quality
to -1. With that sender, expect `received` and `TEST packets` to increase at
1 Hz, `filter` rejections to increase, and `published=0`. This confirms callback
delivery through the public API. A producer supplying valid odometry is needed
to test publication and estimator consumption. No synthetic-data bypass is
included.

## Packet and publication contract

`inc/dsp_link.h` is an unchanged copy of the public header from
`modalai-adsp/src/adsp_proc/qsh_platform/framework/dsp_link/inc/dsp_link.h`.
Keep it synchronized when the API changes. The module imports that API
directly; it contains no QDI, IPCC, shared-memory, or transport retry code.

`inc/odometry_packet.h` describes the current 256-byte CQL ABI-3 odometry
packet. This temporary application-specific layout remains compatible with
the existing CDSP producer. The module checks packet length, magic/version,
commit sequence, producer generation, and forward sequence/sample time.
Initial attachment to a running producer establishes a baseline; previous
packets are not counted as gaps. Latest-sample delivery can legitimately skip
intermediate packets.

The odometry validity checks preserve the prototype's contract: running filter
state (2), nonzero filter generation, one or two cameras with matching flags,
local FRD pose and velocity frames, quality 0 through 100, finite position and
velocity, a unit Hamilton quaternion, and finite nonnegative variances.
Angular velocity may be NaN when unavailable. Zero or future sample timestamps
are rejected, as are repeated/backward sample timestamps within a producer
generation. Sample age is reported without an age-based publication gate.

CDSP supplies the final FRD/Hamilton fields. The receiver copies each field
explicitly, preserves `timestamp_sample` in the shared QTimer microsecond
domain, and sets `timestamp` from SLPI `hrt_absolute_time()` immediately before
publication. No coordinate transformation is applied. The existing QuRT HRT
implementation leaves its clock offset at zero.

Reception and publication run on the `dsp_link` callback thread. There is no
additional polling task or per-packet logging. `stop` closes the descriptor and
waits for callbacks to finish before releasing the uORB advertisement. A failed
close retains the descriptor and callback state so `stop` can be retried. Stop
the module before unloading client code.
