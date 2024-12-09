from band.band_controller import SensorBand
import asyncio
import threading
import time
import socket

def main():
    s = socket.socket()
    port = 5000
    s.connect(('127.0.0.1', port))

    def getcommandfromserver():
        s.send("get_cmd".encode())
        msg = s.recv(1024).decode()
        return msg

    while True:
        
        command = getcommandfromserver()
        print(command)

        time.sleep(0.2)

main()