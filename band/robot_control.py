# Sample code to control a robot, but only printing the commands on screen
# TODO: align axes

import asyncio
import os
import signal
import sys
import numpy as np
#from vpython import box, vector, rate, scene
import numpy as np
import math


current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from band.lib_gforce import gforce


def convert_raw_emg_to_uv(
    data: bytes, resolution: gforce.SampleResolution
) -> np.ndarray[np.float32]:
    min_voltage = -1.25  # volt
    max_voltage = 1.25  # volt

    match resolution:
        case gforce.SampleResolution.BITS_8:
            div = 127.0
            sub = 128
        case gforce.SampleResolution.BITS_12:
            div = 2047.0
            sub = 2048
        case _:
            raise Exception(f"Unsupported resolution {resolution}")

    gain = 1200.0
    conversion_factor = (max_voltage - min_voltage) / gain / div

    emg_data = (data.astype(np.float32) - sub) * conversion_factor

    return emg_data.reshape(-1, len(data))


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
        # imu_box = box(pos=vector(0,0,0), length=2, height=0.5, width=1, color=vector(1,0,0))

        def update_orientation(orientation_data):
            # Assuming orientation_data is a quaternion or Euler angles (roll, pitch, yaw)
            # Replace with the code to transform these angles into the correct orientation
            # Example for Euler angles:
            roll, pitch, yaw = orientation_data
            # imu_box.axis = vector(np.cos(yaw), np.sin(pitch), np.sin(roll))

        await gforce_device.set_motor(True)
        await asyncio.sleep(1)
        await gforce_device.set_motor(False)
        await asyncio.sleep(1)

        await gforce_device.set_subscription(
            gforce.DataSubscription.EULERANGLE
        )        

        # await gforce_device.set_subscription(
        #     gforce.DataSubscription.EMG_RAW
        #     # gforce.DataSubscription.EMG_GESTURE
        # )
        
        q = await gforce_device.start_streaming()
        # q2 = await gforce_device.start_streaming()

        started = 0
        start_roll = 0
        start_pitch = 0
        start_yaw = 0

        while not self.terminated:
            v = await q.get()
            # print(q.qsize())
            # v2 = await q2.get()
            
            # print(v)
            # print(v2)

            if len(v[0]) == 3:
                # Fetch orientation data from your IMU
                orientation_data = v[0]  # Replace with actual data fetching
                # print(orientation_data)
                # continue
                if started == 0:
                    start_roll = orientation_data[1]
                    start_pitch = orientation_data[0]
                    start_yaw = orientation_data[2]
                    started = 1
                if (orientation_data[1] - start_roll) < -20:
                    print("finish")
                    break
                elif (orientation_data[1] - start_roll) > 40:
                    print("finish")
                    break
                else:
                    if (orientation_data[0] - start_pitch) < -40:
                        print("back")
                    elif (orientation_data[0] - start_pitch) > 40:
                        print("forward")
                    if (orientation_data[2] - start_yaw) < -20:
                        print("right")
                    elif (orientation_data[2] - start_yaw) > 20:
                        print("left")
                # print("orientation: ", orientation_data)
            else:
                # emg_data = convert_raw_emg_to_uv(v, gforce_device.resolution)
                # ddp = [abs(i[1] - i[0]) for i in emg_data]
                # print("emg: ", sum(ddp))
                pass
    
            # Update the orientation of the box
            # update_orientation(list(map(math.radians, orientation_data)))
            
            # Control the refresh rate
            # rate(60)  # Adjust for your preferred frame rate
            # await asyncio.sleep(0.5)

        await gforce_device.stop_streaming()
        await gforce_device.disconnect()


if __name__ == "__main__":
    app = Application()
    asyncio.run(app.main())
