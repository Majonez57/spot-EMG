# Code to control a robot
# TODO: align axes

import asyncio
import os
import signal
import sys
import numpy as np
#from vpython import box, vector, rate, scene
import numpy as np
import math
import socket

current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from lib_gforce import gforce

class Application:

    def __init__(self):
        signal.signal(signal.SIGINT, lambda signal, frame: self._signal_handler())
        self.terminated = False

    def _signal_handler(self):
        print("You pressed ctrl-c, exit")
        self.terminated = True


    async def main(self):
        gforce_device = gforce.GForce()

        await gforce_device.connect()
        print("Connected to {0}".format(gforce_device.device_name))

        await gforce_device.set_motor(True)
        await asyncio.sleep(1)
        await gforce_device.set_motor(False)
        await asyncio.sleep(1)

        await gforce_device.set_subscription(
            gforce.DataSubscription.EULERANGLE
        )        
        
        q = await gforce_device.start_streaming()

        started = 0
        start_roll = 0
        start_pitch = 0
        start_yaw = 0

        # get the hostname
        host = socket.gethostname()
        port = 5000  # initiate port no above 1024

        server_socket = socket.socket()  # get instance
        # look closely. The bind() function takes tuple as argument
        server_socket.bind((host, port))  # bind host address and port together

        # configure how many client the server can listen simultaneously
        server_socket.listen(2)
        print("Waiting for connections...")
        conn, address = server_socket.accept()  # accept new connection
        print("Connection from: " + str(address))

        while not self.terminated:
            # receive data stream. it won't accept data packet greater than 1024 bytes
            data = conn.recv(1024).decode()
            print("from connected user: " + str(data))
            if str(data) == "get_cmd":
                msg = ""
                v = await q.get()

                if len(v[0]) == 3:
                    # Fetch orientation data from your IMU
                    orientation_data = v[0]  # Replace with actual data fetching
                    if started == 0:
                        start_roll = orientation_data[1]
                        start_pitch = orientation_data[0]
                        start_yaw = orientation_data[2]
                        started = 1
                    # if (orientation_data[1] - start_roll) < -20:
                    #     print("right")
                    # elif (orientation_data[1] - start_roll) > 40:
                    #     print("left")
                    if (orientation_data[0] - start_pitch) < -40:
                        print("back")
                        msg = "back"
                    elif (orientation_data[0] - start_pitch) > 40:
                        print("forward")
                        msg = "forward"
                    elif (orientation_data[2] - start_yaw) < -20:
                        print("right")
                        msg = "right"
                    elif (orientation_data[2] - start_yaw) > 20:
                        print("left")
                        msg = "left"
                    # print("orientation: ", orientation_data)
                
                conn.send(msg.encode())  # send data to the client

        await gforce_device.stop_streaming()
        await gforce_device.disconnect()
        conn.close()


if __name__ == "__main__":
    app = Application()
    asyncio.run(app.main())
