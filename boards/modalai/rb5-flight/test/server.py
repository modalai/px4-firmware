import socket
import serial
import json
import argparse
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class serverSocket:

	def __init__(self, port):
		self.port = port
		self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.serverSocket.bind(('', self.port))
		self.serverSocket.setblocking(0)
		self.message = ''

	def receive_msgs(self):
		try:
			self.message = self.serverSocket.recv(4096).decode('utf-8')
			return self.message
		except socket.error as e:
			pass

class serialWrite:

	def __init__(self, port, baudrate):
		self.port = port
		self.baudrate = baudrate
		self.ser = serial.Serial(str(self.port), self.baudrate)

	def write(self, string):
		self.ser.write(string)

	def close(self):
		self.ser.close()

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def encode_rc_channel():
	f = fifo()
	rc_channel_values = [65535 for _ in range(18)]
	mavlink_message = mavlink2.MAVLink(f)
	m = mavlink_message.rc_channels_override_encode(
		1,  # target_system
		1,  # target_component
		*rc_channel_values)
	m.pack(mavlink_message)
	encoded_buffer = m.get_msgbuf()
	return encoded_buffer

def encode_radio_status():
	f = fifo()
	mavlink_message = mavlink2.MAVLink(f)
	m = mavlink_message.radio_status_encode(rssi, remrssi, txbuf, noise, remnoise, rxerrors, fixed)
	m.pack(mavlink_message)
	encoded_buffer = m.get_msgbuf()
	return encoded_buffer

def main():
	parser = argparse.ArgumentParser(description='Get parameters for serial and server')
	parser.add_argument('--server_port', '-s', type=int, required=True)
	parser.add_argument('--serial_port', '-p', type=str, required=True)
	parser.add_argument('--serial_baudrate', '-b', type=int, required=True)

	args = parser.parse_args()

	writer = serialWrite(args.serial_port, args.serial_baudrate)
	server = serverSocket(args.server_port)

	while True:
		data = server.receive_msgs()
		if(data):
			print(data)
		time.sleep(1)

if __name__ == "__main__":
	main()
