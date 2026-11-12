# SPDX-License-Identifier: BSD-3-Clause

px4_add_module(
	MODULE drivers__modalai__dsp_link
	MAIN dsp_link
	INCLUDES
		inc
	SRCS
		src/dsp_link.cpp
)
