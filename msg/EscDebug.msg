# Raw ESC debug packet
# The whole packet (header, length, type, id, payload, crc) is saved for offline parsing

uint64 timestamp    # [us] Time since system start

uint8 esc_id        # hardware ID of the ESC that sent the packet
uint8 len           # number of valid bytes in data
uint8[64] data      # raw packet bytes, starting with the header byte

uint8 ORB_QUEUE_LENGTH = 8
