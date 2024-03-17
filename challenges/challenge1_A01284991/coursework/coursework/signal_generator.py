import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import numpy as np

class Signal_Pub(Node):
    def __init__(self): 

        super().__init__('signal_generator')

        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher = self.create_publisher(Float32, 'time', 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('signal_generator node successfully initialized!!')

        self.signal = Float32()
        self.time = Float32()
        self.time.data = 0.0

    def timer_callback(self):

        self.signal.data = np.sin(self.time.data)
        self.time.data += 0.1 

        self.signal_publisher.publish(self.signal)
        self.time_publisher.publish(self.time)


def main(args=None):
    rclpy.init(args=args)
    m_p = Signal_Pub()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
