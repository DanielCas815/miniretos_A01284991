import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import numpy as np

class Process_Sub(Node):    

    def __init__(self):

        super().__init__('process')
        
        self.time_subscriber = self.create_subscription(Float32, 'time', self.time_callback, 10)
        self.signal_subscriber = self.create_subscription(Float32, 'signal', self.signal_callback, 10)

        self.new_signal_publisher = self.create_publisher(Float32, 'proc_signal', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Listener node initialized!!!')

        self.n_time = Float32()
        self.n_signal = Float32()
        
    def time_callback(self, time: Float32):
        self.n_time.data = np.sin(time.data + 4)/2 + 3 # phase shift, offset and applitude reduction
        self.get_logger().info("New Signal = " + str(self.n_time.data))

    def signal_callback(self, signal: Float32):
        self.n_signal.data = signal.data/2 + 3 # offset and applitude reduction
        self.get_logger().info("New Signal = " + str(self.n_signal.data))
    
    def timer_callback(self):
        self.new_signal_publisher.publish(self.n_time)
        

def main(args=None):
    rclpy.init(args=args)
    m_p = Process_Sub()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()