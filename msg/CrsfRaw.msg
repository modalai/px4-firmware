# Raw CRSF frame data for bridging between the CRSF UART and external services
# crsf_raw_rx: Data received from the CRSF receiver
# crsf_raw_tx: Data to be transmitted to the CRSF receiver

uint64 timestamp                    # time since system start (microseconds)

uint8 len                           # Number of valid bytes in data
uint8[64] data                      # Complete raw CRSF frame

uint8 ORB_QUEUE_LENGTH = 8

# TOPICS crsf_raw_rx crsf_raw_tx
