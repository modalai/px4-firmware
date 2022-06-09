import socket
import time
pings = 1
#Send ping 10 times
while pings < 11:

    #Create a UDP socket
    clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    #Set a timeout value of 1 second
    clientSocket.settimeout(1)

    #Ping to server
    message = 'test'.encode('utf-8')

    addr = ("127.0.0.1", 8000)
    clientSocket.sendto(message, addr)
    pings+=1
    time.sleep(1)
