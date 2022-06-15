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

def encode_rc_channel(json_data):
	f = fifo()
	mavlink_message = mavlink2.MAVLink(f)
	m = mavlink_message.rc_channels_override_encode(**json_data)
	m.pack(mavlink_message)
	encoded_buffer = m.get_msgbuf()
	return encoded_buffer

def encode_radio_status(json_data):
	f = fifo()
	mavlink_message = mavlink2.MAVLink(f)
	m = mavlink_message.radio_status_encode(**json_data)
	m.pack(mavlink_message)
	encoded_buffer = m.get_msgbuf()
	return encoded_buffer

def main():

	parser = argparse.ArgumentParser(description='Get parameters for serial and server')
	parser.add_argument('--server_port', '-s', type=int, required=True)
	parser.add_argument('--serial_port', '-p', type=str, required=True)
	parser.add_argument('--serial_baudrate', '-b', type=int, required=True)
	parser.add_argument('--rc_channel_json', '-rc', type=str, required=False)
	parser.add_argument('--radio_status_json', '-rs', type=str, required=False)
	parser.add_argument('--rate', '-r', type=int, required=True)

	args = parser.parse_args()
	hz_rate =   1 / args.rate
	encoded_rc_json = None
	encoded_radio_status_json = None

	if(args.rc_channel_json):
		rc_channel_json = json.load(open(args.rc_channel_json))
		encoded_rc_json = encode_rc_channel(rc_channel_json['mavlink_message'])

	if(args.radio_status_json):
		radio_status_json = json.load(open(args.radio_status_json))
		encoded_radio_status_json = encode_radio_status(radio_status_json['mavlink_message'])

	writer = serialWrite(args.serial_port, args.serial_baudrate)
	server = serverSocket(args.server_port)
	counter = 0

	while True:
		data = server.receive_msgs()
		try:
			if(data):
				json_data = json.loads(data)
				if(json_data):
					if(json_data['message_type'] == 'rc_channel'):
						if('parameter_name' in json_data and 'parameter_value' in json_data):
							rc_channel_json['mavlink_message'][json_data["parameter_name"]] = int(json_data["parameter_value"])
							encoded_rc_json = encode_rc_channel(rc_channel_json['mavlink_message'])
						else:
							encoded_rc_json = encode_rc_channel(json_data['mavlink_message'])

					if(json_data['message_type'] == 'radio_status'):
						if('parameter_name' in json_data and 'parameter_value' in json_data):
							radio_status_json['mavlink_message'][json_data["parameter_name"]] = int(json_data["parameter_value"])
							encoded_radio_status_json = encode_radio_status(radio_status_json)
						else:
							encoded_rc_json = encode_radio_status(json_data['mavlink_message'])

				else:
					data = None

			if(encoded_radio_status_json or encoded_rc_json):
				if(encoded_radio_status_json):
					writer.write(encoded_radio_status_json)

				if(encoded_rc_json):
					writer.write(encoded_rc_json)

		except Exception as e:
			print(e)
			pass

		time.sleep(hz_rate)

if __name__ == "__main__":
	main()
