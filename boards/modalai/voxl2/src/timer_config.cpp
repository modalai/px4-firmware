
// Placeholder

#include <px4_arch/io_timer_hw_description.h>

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
