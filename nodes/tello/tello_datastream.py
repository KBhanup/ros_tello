

import socket
import threading

## Class to handle the returned data stream from the tello
# This class is instantiatied when Tello.start_data_stream is called.
class TelloDataStream:

    ## Begin listening for the datastream.
    #  @param self This object pointer.
    #  @param stream_callback Callback to receive data from the stream: F(data: dict, ip: str)
    #  @param local_ip Ip to subscribe on. This should always be the local address (default: '0.0.0.0')
    def __init__(self, stream_callback,local_ip="0.0.0.0"):
        self.stream_callback=stream_callback
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print(f"Datastream Binding to {local_ip}:8890")

        self.socket.bind((local_ip, 8890))

        self.running = True
        self.receive_state_thread = threading.Thread(target=self._receive_state_thread)
        self.receive_state_thread.daemon = True
        self.receive_state_thread.start()

    ## Ensure the thread is killed
    #  @param self This object pointer.
    def __del__(self,):
        self.stop()

    ## Thread to spin up to receive the data stream.
    #  @param self This object pointer.
    def _receive_state_thread(self):
        while self.running:
            # Checking for Tello response, throws socket error
            try:
                stateresponse, ip = self.socket.recvfrom(1024)
                self.stream_callback(self.state_to_dict(stateresponse.decode("utf-8")), ip)
            except socket.error as exc:
                print(f'Socket error: {exc}')

    ## Close the socket, stop the thread, and join the thread.
    #  @param self This object pointer.
    def stop(self):
        self.socket.close()
        self.running = False
        self.receive_state_thread.join()

    ## Converts a data stream string to a dictionary
    #  @param self This object pointer.
    #  @param data Data stream string to be converted.
    def state_to_dict(self,data):
        splt = data.split(";")
        d= {}
        for i in splt:
            e = i.split(":")
            try:
                d[e[0]] = float(e[1])
            except:
                continue
        return d
    
        
