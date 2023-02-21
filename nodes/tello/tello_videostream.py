import socket
import threading
import cv2
from time import sleep

## Class to handle the returned video stream from the tello
# This class is instantiatied when Tello.start_video_stream is called.
class TelloVideoStream:

    ## Begin listening for the video stream.
    #  @param self This object pointer.
    #  @param callback Callback to receive data from the stream: F(frame:numpy.array)
    #  @param local_ip Ip to subscribe on (default: '0.0.0.0'). This should always be the local address.
    def __init__(self, callback,local_ip="0.0.0.0"):
        self.callback = callback
        self.local_ip=local_ip
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_video_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    ## Ensure the thread is killed
    #  @param self This object pointer.
    def __del__(self,):
        self.stop()

    ## Thread to spin up to receive the video stream.
    #  @param self This object pointer.
    def receive_video_thread(self):
        print("Video thread begun")
        print(f"Video Binding to {self.local_ip}:11111")
        cap = cv2.VideoCapture(f'udp://{self.local_ip}:11111')
        while not cap.isOpened():
            sleep(.1)
        print("Capture opened")
        while self.running:
            #print("Attempting frame read")
            ret, frame = cap.read()
            self.callback(frame)
        print("Closing Capture")
        cap.release()

    ## Close the socket, stop the thread, and join the thread.
    #  @param self This object pointer.
    def stop(self):
        self.running = False
        self.receive_video_thread.join()
