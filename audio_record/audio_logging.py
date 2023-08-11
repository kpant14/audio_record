# Quickly import essential libraries
import queue
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.io.wavfile import write
from audio_msgs.msg import AudioSignal

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.q =queue.Queue()
        self.subscription = self.create_subscription(
                AudioSignal,
                'audio_signals',
                self.listener_callback,
                10)

    def listener_callback(self, msg):
        n_mics = msg.n_mics
        n_buffer = msg.n_buffer
        data = np.array(msg.signals_vect).reshape((n_buffer, n_mics))
        self.q.put(data.copy())

    def save_wav(self):
        self.save_data = []
        while self.q.qsize():
            self.save_data.append(self.q.get())
        self.save_data = np.concatenate(self.save_data)
        for i in range(self.save_data.shape[1]):
            fname = f"{self.get_name()}_mic{i}.wav"
            write(fname, 48000, self.save_data[:,i])    
            self.get_logger().info(f"Saved audio as {fname}")

def main(args=None):
    rclpy.init(args=args)

    audio_subscriber = AudioSubscriber()
    try:
        rclpy.spin(audio_subscriber)
    except KeyboardInterrupt:
        audio_subscriber.save_wav()
        

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    audio_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
