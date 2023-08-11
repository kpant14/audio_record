import time
import rclpy
from rclpy.node import Node
import threading
import numpy as np
import sounddevice as sd
import queue
from audio_msgs.msg import AudioSignal
from std_msgs.msg import Header
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        
class AudioPublisher(Node):

    def __init__(self):
        super().__init__('audio_publisher')
        timer_cb_grp = MutuallyExclusiveCallbackGroup()
        
        # Device and channel setup
        self.device = 'hw:1,0' 
        self.channels = [1,2,3,4,5,6,7] # a list of audio channels
        self.device_info =  sd.query_devices(self.device, 'input')
        self.get_logger().info('%s' % self.device_info)

        self.samplerate = self.device_info['default_samplerate']
        # Start a stream to capture raw data from the device
        self.stream  = sd.InputStream( device = self.device, 
                                    channels = max(self.channels), 
                                    samplerate = self.samplerate, 
                                    callback  = self.audio_callback)
        self.stream.start()

        self.publisher_ = self.create_publisher(AudioSignal, 'audio_signals', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=timer_cb_grp)
    
        self.q = queue.Queue() 

    def audio_callback(self, indata,frames,time,status):
        self.q.put(indata.copy())

    def timer_callback(self):
        if not self.q.empty(): 
            # Filling the message 
            msg = AudioSignal()
            msg.header = Header()
            msg.fs = int(self.samplerate)
            msg.n_mics = len(self.channels)
            data = 10*self.q.get_nowait()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.n_buffer = data.shape[0]
            msg.signals_vect = data.flatten().tolist()
            self.publisher_.publish(msg)
        
def main(args=None):

    rclpy.init(args=args)
    
    audio_publisher = AudioPublisher()
    rclpy.spin(audio_publisher)
    audio_publisher.stream.close()
    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # audio_publisher.destroy_node()
    rclpy.shutdown()

    # Lets define audio variables


if __name__ == '__main__':
    main()
