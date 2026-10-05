/****************************************************************************
 *
 *   Copyright (c) 2026 ModalAI, Inc. All rights reserved.
 *
 ****************************************************************************/

/*
 * VOXL: keep PX4 from stepping the Linux wall clock (CLOCK_REALTIME).
 *
 * On VOXL the apps-side PX4 shares CLOCK_REALTIME with the camera stack, and
 * CamX times its per-request waits against it: a step forward of more than
 * about a second makes every streaming camera's pending wait time out, CamX
 * flushes and resets each pipeline, and occasionally a camera never resumes.
 * PX4 itself only needs UTC in sensor_gps / ULog, which is unaffected.
 *
 * Linked into px4 with -Wl,--wrap=clock_settime (cmake/link_libraries.cmake),
 * so every PX4 call (gps setClock, mavlink SYSTEM_TIME, uxrce-dds) lands here.
 * Mirrors px4_clock_settime() on QuRT, which is already a no-op on the DSP.
 */

#include <time.h>

int __real_clock_settime(clockid_t clk_id, const struct timespec *tp);

int __wrap_clock_settime(clockid_t clk_id, const struct timespec *tp)
{
	if (clk_id == CLOCK_REALTIME) {
		// report success so callers (gps, mavlink SYSTEM_TIME) don't log errors
		return 0;
	}

	return __real_clock_settime(clk_id, tp);
}
