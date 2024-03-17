import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from params_msgs.msg import Params

from numpy import sin as np_sin
from scipy import signal as sig


class Signal_recon(Node):
 
    def __init__(self):
        super().__init__('signal_reconstruction')

        #Node initialization message
        self.get_logger().info('signal_reconstruction node initialized!!!')

        #Topics 
        #-----------------------------------------------------------------------------
        self.signal_params_sub = self.create_subscription(Params, 'signal_params', self.signal_params_callback, 10)

        self.signal_reconstructed_pub = self.create_publisher(Float32, 'signal_reconstructed', 10)
        #-----------------------------------------------------------------------------

        #Timer for Monitor callback function
        self.timer_period_1 = 0.001
        self.timer1 = self.create_timer(self.timer_period_1, self.timer_callback_M)

        #Variables
        self.signal = Float32()
        
    def signal_params_callback(self, parm: Params):
        #Signal reconstruction with Param interface values
        if(parm.wave_type == 1):
            self.signal.data = (parm.amplitude * np_sin(parm.frequency * parm.time)) + parm.offset
        elif(parm.wave_type == 2):
            self.signal.data = (parm.amplitude * sig.square(parm.frequency * parm.time)) + parm.offset
        else:
            self.signal.data = (parm.amplitude * sig.sawtooth(parm.frequency * parm.time/10)) + parm.offset

    def timer_callback_M(self):
        self.signal_reconstructed_pub.publish(self.signal)


def main(args=None):
    rclpy.init(args=args)
    m_p = Signal_recon()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()