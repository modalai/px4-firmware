# Raw ESC debug packet.
# The ESC sends it instead of a feedback packet: once right after a motor stop (with the state latched
# before the stop), once after boot, and periodically (every 100th feedback packet) with the live state.
# The whole packet (header, length, type, id, payload, crc) is saved for offline parsing (EscStopDebug_t).

uint64 timestamp	# [us] Time since system start

uint8 esc_id		# hardware ID of the ESC that sent the packet
uint8 len		# number of valid bytes in data
uint8[64] data		# raw packet bytes, starting with the header byte

uint8 ORB_QUEUE_LENGTH = 8
