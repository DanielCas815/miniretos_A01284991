import rclpy
from rclpy.node import Node
from rclpy.node import Parameter
from std_msgs.msg import Float32
from params_msgs.msg import Params
import numpy as np
from scipy import signal as sig

class Signal_generation(Node):
    
    def __init__(self):
        super().__init__('signal_generator') 

        #Node initalization message
        self.get_logger().info('signal_generator node successfully initialized!!')

        #Parameters
        self.declare_parameters(
            namespace='', 
            parameters=[ 
                ('wave_type', rclpy.Parameter.Type.INTEGER),
                ('wave_time', rclpy.Parameter.Type.DOUBLE),
                ('sine_wave.frequency', rclpy.Parameter.Type.DOUBLE),
                ('sine_wave.offset', rclpy.Parameter.Type.DOUBLE),
                ('sine_wave.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('square_wave.frequency', rclpy.Parameter.Type.DOUBLE),
                ('square_wave.offset', rclpy.Parameter.Type.DOUBLE),
                ('square_wave.amplitude', rclpy.Parameter.Type.DOUBLE),
                ('saw_wave.frequency', rclpy.Parameter.Type.DOUBLE),
                ('saw_wave.offset', rclpy.Parameter.Type.DOUBLE),
                ('saw_wave.amplitude', rclpy.Parameter.Type.DOUBLE)
        ])

        #Topics
        #-----------------------------------------------------------------------------
        #Publisher for signal in Monitor
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)

        #Publisher for custom msg Params
        self.signal_param_publisher = self.create_publisher(Params, 'signal_params', 10)
        #-----------------------------------------------------------------------------

        #Timer for Monitor callback function
        self.timer_period_1 = 0.001
        self.timer1 = self.create_timer(self.timer_period_1, self.timer_callback_M)

        #Timer for Param callback function
        self.timer_period_2 = 0.1
        self.timer2 = self.create_timer(self.timer_period_2, self.timer_callback_P)

        #Variables
        self.signal = Float32()
        self.time = 0.0
        self.type = 0
        self.amplitude = 0.0
        self.frequency = 0.0
        self.offset = 0.0
        
        self.signal_params = Params()


    def timer_callback_M(self):
        #Check the type of signal to generate and publish
        self.type = self.get_parameter('wave_type').get_parameter_value().integer_value
        #self.time = self.get_parameter('wave_time').get_parameter_value().double_value
        if(self.type == 1):
            self.frequency = self.get_parameter('sine_wave.frequency').get_parameter_value().double_value
            self.offset = self.get_parameter('sine_wave.offset').get_parameter_value().double_value
            self.amplitude = self.get_parameter('sine_wave.amplitude').get_parameter_value().double_value
            self.signal.data = (self.amplitude * np.sin(self.frequency*self.time)) + self.offset
        elif(self.type == 2):
            self.frequency = self.get_parameter('square_wave.frequency').get_parameter_value().double_value
            self.offset = self.get_parameter('square_wave.offset').get_parameter_value().double_value
            self.amplitude = self.get_parameter('square_wave.amplitude').get_parameter_value().double_value
            self.signal.data = (self.amplitude * sig.square(self.frequency*self.time)) + self.offset
        else:
            self.frequency = self.get_parameter('saw_wave.frequency').get_parameter_value().double_value
            self.offset = self.get_parameter('saw_wave.offset').get_parameter_value().double_value
            self.amplitude = self.get_parameter('saw_wave.amplitude').get_parameter_value().double_value
            self.time += 1/self.frequency
            self.signal.data = (self.amplitude * sig.sawtooth(self.frequency * self.time/10)) + self.offset

        self.time = self.time + 0.001
        self.signal_publisher.publish(self.signal)


    def timer_callback_P(self):
        #Publish signal parameters in custom message topic
        self.signal_params.wave_type = self.type
        self.signal_params.time = self.time
        self.signal_params.frequency = self.frequency
        self.signal_params.offset = self.offset
        self.signal_params.amplitude = self.amplitude

        self.signal_param_publisher.publish(self.signal_params)



def main(args=None):
    rclpy.init(args=args)
    node = Signal_generation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()