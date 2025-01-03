# Sample code to get data from the sensor, print it on the terminal and show a 3d model following sensor orientation in a browser
# TODO: align axes

import asyncio
import os
import signal
import sys
import numpy as np
from vpython import box, vector, rate, scene
import math
import pyvista as pv
from pyvista import examples
from scipy.spatial.transform import Rotation as R


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

        # Create a 3D box to represent the IMU's orientation
        imu_box = box(pos=vector(0,0,0), length=2, height=0.5, width=1, color=vector(1,0,0))

        def update_orientation(orientation_data):
            # Assuming orientation_data is a quaternion or Euler angles (roll, pitch, yaw)
            # Replace with the code to transform these angles into the correct orientation
            # Example for Euler angles:
            roll, pitch, yaw = orientation_data
            imu_box.axis = vector(np.cos(yaw), np.sin(pitch), np.sin(roll))

        await gforce_device.set_motor(True)
        await asyncio.sleep(2)
        await gforce_device.set_motor(False)
        await asyncio.sleep(2)

        await gforce_device.set_subscription(
            gforce.DataSubscription.EULERANGLE
        )

        q = await gforce_device.start_streaming()

        while not self.terminated:
            v = await q.get()
            print(v)

            # Fetch orientation data from your IMU
            orientation_data = v[0]  # Replace with actual data fetching
    
            # Update the orientation of the box
            update_orientation(list(map(math.radians, orientation_data)))
            
            # Control the refresh rate
            rate(60)  # Adjust for your preferred frame rate

        await gforce_device.stop_streaming()
        await gforce_device.disconnect()


if __name__ == "__main__":
    app = Application()
    asyncio.run(app.main())
