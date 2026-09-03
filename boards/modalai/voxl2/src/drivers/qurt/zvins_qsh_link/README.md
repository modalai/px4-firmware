# PX4 ZVINS QSH receiver

The canonical build, signing, flash, rollback, and board acceptance guide is
the `README.md` at the root of the multi-repository integration checkout
(`KERNEL_MAN_WORKTREES`). This module is carried upstream through the
`voxl-mainline-fpv-px4` wrapper on its `zmai-future` branch; the wrapper's
`sign-slpi.sh` performs the QSH signing step described below.

This module is the ADSP v66 endpoint of the direct QCS6490 ZVINS2 result path.
It receives final float, FRD `VehicleOdometry` fields from the CDSP v68 and
publishes `vehicle_visual_odometry` inside PX4's `sensor_process`.

```text
CDSP user PD -> fixed SMEM ring -> CDSP root QDI RING -> MPROC IPCC
             -> ADSP root fused WAIT_RX_ACK -> this PX4 task -> uORB
```

Linux/APSS does not map, relay, address, start, or timestamp the result. The
PX4 endpoint starts after `uorb` and before `qshell` from
`platforms/qurt/src/px4/main.cpp`.

## Fixed contract

| Item | Value |
| --- | --- |
| QDI root service | `/dev/zql_transport` |
| Transport/wire ABI | 1 / 3 |
| Fingerprint | `0xac6c7156` |
| SMEM | item 136, 8,320 bytes |
| Doorbell | MPROC, CDSP client 6 to LPASS client 3, signal 3 |
| Producer/consumer | CDSP v68 / ADSP v66 |

The receiver normally blocks in the ADSP root service's fused `WAIT_RX_ACK`
operation. That one QDI invocation waits on and acknowledges the root-owned
IPCC wake before PX4 reads the latest commit-last slot. There is no generic
IPCC callback worker and no timed poll in the healthy path. `BIND_RX` also
returns a remote signal for compatibility with an older ABI-1 root, but a
matched production run must not use that legacy `WAIT` + `CLEAR` path.
`WAKE_RX` makes shutdown deterministic. If the fixed root service is
unavailable, a timed control wait retries the bind/map operation without
reading SMEM; result publication remains disabled. The data plane has no
polling fallback.

CDSP already performs the unchanged native-FLU/JPL to local-FRD/Hamilton
publication transform. This module does not transform or remap clocks. It
validates the slot and final float payload, copies the
`VehicleOdometry` fields, preserves `timestamp_sample`, sets
`timestamp = hrt_absolute_time()`, and rejects future samples. Sensor age is
reported but is not a publication gate. The 33.333 ms requirement is scored
against the arrival of the next message: 30 Hz is the target and 20 Hz (a
50 ms interval) is the lowest acceptable sustained publication rate. Because
the preceding message's interval is only known when its successor arrives, a
late interval increments diagnostics but never causes the recovery sample to
be dropped.

Build the real PX4 v1.18 APSS/QSH targets from the PX4 tree root in
ModalAI's build environment with diagnostic timing disabled:

```bash
docker run --rm \
  -v "$PWD:/usr/local/workspace" \
  -w /usr/local/workspace \
  rb5-flight-px4-build-docker:v1.4 \
  bash -lc 'source /home/build-env.sh; make modalai_voxl2 -j8'
grep -Fx 'ZQL_DIAGNOSTIC_HOP_TIMING:BOOL=OFF' \
  build/modalai_voxl2_slpi/CMakeCache.txt
```

From the `voxl-mainline-fpv-px4` wrapper, `./build.sh` runs the same build and
performs that cache check itself.

The raw QSH output below is an unsigned intermediate and must never be
flashed:

```text
build/modalai_voxl2_slpi/platforms/qurt/libpx4.so
```

Sign and validate that ELF with Hexagon SDK 6.6.0 (`./sign-slpi.sh` in the
wrapper writes `build/modalai_voxl2_slpi/platforms/qurt/signed/output/libpx4.so`
and `make_package.sh` packages that file when it exists), then deploy only the
signed output. The exact signing and matched-set deployment commands are in the
canonical guide named above.

QShell is diagnostic only:

```text
zvins_qsh_link status
listener vehicle_visual_odometry 5
```

A healthy status has an active root bind, advancing doorbell/publication and
`fused_waits` counters, `legacy_waits=0`, `data_polls=0`, no recovery waits or
transport/payload/timestamp failures, steady-state intervals near the 30 Hz
target, and no `below_20hz` violations. Inspect and report `late_30hz`; small
scheduling excursions above 33,334 us are diagnostic rather than dropped
samples.

The ZVINS and PX4 copies of `zvins_qsh_link_wire.[ch]` and
`zvins_qsh_link_qdi.[ch]` must remain byte-identical. The full architecture,
clock derivation, root-firmware ownership, ABI layout, build matrix, deployment
rule, and acceptance checklist are in the ZVINS worktree's
`doc/ZVINS_QSH_LINK.md`.
