# Code to control a robot
# TODO: align axes

import asyncio
import os
import signal
import sys
import numpy as np
import numpy as np
import math


current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from band.lib_gforce import gforce


class SensorBand:

    def __init__(self):
        signal.signal(signal.SIGINT, lambda signal, frame: self._signal_handler())
        self.terminated = False
        self.started = 0
        self.start_roll = 0
        self.start_pitch = 0
        self.start_yaw = 0

    def _signal_handler(self):
        print("You pressed ctrl-c, exit")
        self.terminated = True

    async def get_cmd(self):
        v = await self.q.get()

        #with self.q.mutex:
        #    self.q.clear()
        
        if len(v[0]) == 3:
            # Fetch orientation data from your IMU
            orientation_data = v[0]  # Replace with actual data fetching
            if self.started == 0:
                self.start_roll = orientation_data[1]
                self.start_pitch = orientation_data[0]
                self.start_yaw = orientation_data[2]
                self.started = 1
            # if (orientation_data[1] - start_roll) < -20:
            #     print("right")
            # elif (orientation_data[1] - start_roll) > 40:
            #     print("left")
            if (orientation_data[0] - self.start_pitch) < -40:
                print("back")
                return "back"
            elif (orientation_data[0] - self.start_pitch) > 40:
                print("forward")
                return "forward"
            if (orientation_data[2] - self.start_yaw) < -20:
                print("right")
                return "right"
            elif (orientation_data[2] - self.start_yaw) > 20:
                print("left")
                return "left"
            # print("orientation: ", orientation_data)

    async def start(self):
        self.gforce_device = gforce.GForce()

        await self.gforce_device.connect()
        print("Connected to {0}".format(self.gforce_device.device_name))

        await self.gforce_device.set_motor(True)
        await asyncio.sleep(1)
        await self.gforce_device.set_motor(False)
        await asyncio.sleep(1)

        await self.gforce_device.set_subscription(
            gforce.DataSubscription.EULERANGLE
        )        
        
        self.q = await self.gforce_device.start_streaming()

    async def stop(self):
        await self.gforce_device.stop_streaming()
        await self.gforce_device.disconnect()


if __name__ == "__main__":
    app = SensorBand()
    asyncio.run(app.start())
