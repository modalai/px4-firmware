import socket
import serial
import json
import argparse
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class clientSocket:

    def __init__(self, port):
        self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_addr = ("", port)

    def send_msgs(self, message):
        try:
            self.clientSocket.sendto(message.encode('utf-8'), self.server_addr)
        except socket.error as e:
            pass

def main():

    parser = argparse.ArgumentParser(description='Send data to serial port')
    parser.add_argument('--server_port', '-s', type=int, required=True)
    parser.add_argument('--rc_channel_json', '-rc', action='store_true', required=False)
    parser.add_argument('--radio_status_json', '-rs', action='store_true', required=False)
    parser.add_argument('--rc_json_file', '-rcj', type=str, required=False)
    parser.add_argument('--rs_json_file', '-rsj', type=str, required=False)
    parser.add_argument('--parameter_name', '-pn', type=str, required=False)
    parser.add_argument('--parameter_value', '-pv', type=str, required=False)

    args = parser.parse_args()
    client = clientSocket(args.server_port)

    data_dict ={}
    if(args.rc_channel_json or args.radio_status_json):
        if(args.rc_channel_json and args.parameter_name and args.parameter_value):
            data_dict["message_type"] = "rc_channel"
            data_dict["parameter_name"] = str(args.parameter_name)
            data_dict["parameter_value"] = str(args.parameter_value)
        elif(args.rc_channel_json and args.rc_json_file):
            data_dict['message_type'] = 'rc_channel'
            data_dict['mavlink_message'] = json.load(open(args.rc_json_file))['mavlink_message']

        if(args.radio_status_json and args.parameter_name and args.parameter_value):
            data_dict["message_type"] = "radio_status"
            data_dict["parameter_name"] = str(args.parameter_name)
            data_dict["parameter_value"] = str(args.parameter_value)
        elif(args.radio_status_json and args.rs_json_file):
            data_dict['message_type'] = 'radio_status'
            data_dict['mavlink_message'] = json.load(open(args.rs_json_file))['mavlink_message']

        client.send_msgs(json.dumps(data_dict))

if __name__ == "__main__":
    main()
