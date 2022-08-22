
// Placeholder

#include <px4_arch/io_timer_hw_description.h>

/* Timer allocation
 *
 * TIM5_CH4  T FMU_CH1
 * TIM5_CH3  T FMU_CH2
 * TIM5_CH2  T FMU_CH3
 * TIM5_CH1  T FMU_CH4
 *
 * TIM4_CH2  T FMU_CH5
 * TIM4_CH3  T FMU_CH6
 *
 * TIM12_CH1 T FMU_CH7
 * TIM12_CH2 T FMU_CH8
 *
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer5, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
	initIOTimer(Timer::Timer12),
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer2),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel4}, {GPIO::PortI, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortH, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortH, GPIO::Pin11}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortH, GPIO::Pin10}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel1}, {GPIO::PortH, GPIO::Pin6}),
	initIOTimerChannel(io_timers, {Timer::Timer12, Timer::Channel2}, {GPIO::PortH, GPIO::Pin9}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
