import zmq
import json
import time
import numpy as np
import signal

# Initialize ZMQ publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.setsockopt(zmq.LINGER, 0)  # Ensures the socket closes immediately
socket.bind("tcp://*:5555")

# Graceful exit function
def clean_exit(signum, frame):
    print(f"\nReceived signal {signum}. Cleaning up...")
    socket.close()  # Close the socket properly
    context.term()  # Terminate ZMQ context
    print("ZMQ socket closed. Exiting.")
    exit(0)

# Register signal handlers for Ctrl+C (SIGINT) and Ctrl+Z (SIGTSTP)
signal.signal(signal.SIGINT, clean_exit)   # Handle Ctrl+C
signal.signal(signal.SIGTSTP, clean_exit)  # Handle Ctrl+Z

def publish_basilisk_data():
    try:
        while True:
            msg = {
                "time": time.time(),
                "position": np.random.rand(3).tolist(),
                "velocity": np.random.rand(3).tolist(),
            }
            socket.send_string(json.dumps(msg))
            print(f"Published: {msg}")
            time.sleep(.01)

    except Exception as e:
        print(f"Error: {e}")
        

# Start publishing data
publish_basilisk_data()