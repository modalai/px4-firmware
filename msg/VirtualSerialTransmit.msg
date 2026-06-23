uint8 ORB_QUEUE_LENGTH = 32
uint8 SERIAL_CONTROL_DATA_LENGTH = 70
uint8 DEVICE_ID_MIN = 100
uint8 DEVICE_ID_MAX = 109

uint64 timestamp
uint32 baudrate
uint16 timeout
uint16 sequence
uint8 device
uint8 flags
uint8 data_length
uint8[70] data
