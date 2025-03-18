import zmq
import json
import time
import numpy as np
import signal

class HandlerFun():
    def __init__(self):
        super(HandlerFun,self).__init__()
        # ZMQ Context - one is sufficient:
        self.context = zmq.Context()
        
        # Send port
        self.port = 5555
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")
        
        signal.signal(signal.SIGINT, lambda s, f: self.clean_exit(s, f))   # Handle Ctrl+C
        signal.signal(signal.SIGTSTP, lambda s, f: self.clean_exit(s, f))  # Handle Ctrl+Z
        
        # Receive port:
        self.port_2 = 7070
        self.socket_2 = self.context.socket(zmq.SUB)
        self.socket_2.connect(f"tcp://localhost:{self.port_2}")
        self.socket_2.setsockopt_string(zmq.SUBSCRIBE, "")
        # self.sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # Timeout to prevent blocking forever
    
        
    def clean_exit(self, signum, frame):
        print(f"\nReceived signal {signum}. Cleaning up...")
        self.socket.close()  # Close the socket properly
        self.socket_2.close()  # Close the socket properly
        self.context.term()  # Terminate ZMQ context
        print("ZMQ socket closed. Exiting.")
        exit(0)
    
    
    def publish_basilisk_data(self):
        try:
            while True:
                fake_msg = {
                "time": time.time(),
                "position": np.random.rand(3).tolist(), # To check if we need `tolist()`!
                "velocity": np.random.rand(3).tolist(),
                "attitude": np.random.rand(3).tolist(),
                "omega": np.random.rand(3).tolist(),
                }
                
                self.socket.send_string(json.dumps(fake_msg))
                # print(f"Published: {fake_msg}")
                time.sleep(1)
                
                try:
                    msg_recv = self.socket_2.recv_string(flags=zmq.NOBLOCK)
                    if msg_recv:
                        print(f"Received from ROS2: {msg_recv}")
                    ROS2_raw_data_json = json.loads(msg_recv)
                    
                    
                except zmq.Again:
                    pass

        except Exception as e:
            print(f"Error: {e}")

A = HandlerFun()
A.publish_basilisk_data()