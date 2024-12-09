from band.band_controller import SensorBand
import asyncio
import threading
import time

def connect_to_band():
    band = None
    while True:
        try:
            band = SensorBand()
            print("Intialising Band...")
            asyncio.run(band.start())
            print("Initiating Robot....")
            return band
        except:
            print("Failed to connect to band... Retrying")

def main():
    band = connect_to_band()

    def thr(loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()
    
    loop = asyncio.new_event_loop()
    t = threading.Thread(target=thr, args=(loop, ), daemon=True)
    t.start()
    time.sleep(2)
    while True:
        
        command = asyncio.run_coroutine_threadsafe(band.get_cmd(), loop).result()
        print(command)

        time.sleep(0.6)

main()