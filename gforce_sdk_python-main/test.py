from lib_gforce.gforce import GForceProfile, GForceData
import threading

def on_data_handler(data):
    if data.type == GForceData.EMG:
        print(f"EMG Data: {data.value}")
    elif data.type == GForceData.EULER:
        print(f"Euler Angles: Pitch: {data.value.pitch}, Yaw: {data.value.yaw}, Roll: {data.value.roll}")

def main():
    # Initialize and connect to the gForce Armband
    gforce = GForceProfile()
    gforce.connect()

    # Set up data handling
    gforce.set_data_handler(on_data_handler)

    # Start data streaming
    gforce.start_data_streaming()

    print("Press ENTER to stop the program...")
    input()  # Wait for ENTER key press

    # Stop streaming and disconnect
    gforce.stop_data_streaming()
    gforce.disconnect()

if __name__ == "__main__":
    main()
