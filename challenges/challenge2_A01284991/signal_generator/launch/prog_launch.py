import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('signal_generator'),
        'config',
        'params.yaml'
    )

    signal_generator_node = Node(
        package = 'signal_generator',
        executable = 'signal_generator',
        output = 'screen',
        name='signal_generator',
        parameters = [config] 
    )

    signal_reconstruction_node = Node(
        package = 'signal_generator',
        executable = 'signal_reconstruction',
        output = 'screen',
    )

    rqt_graph_node = Node(
        package = 'rqt_graph',
        executable = 'rqt_graph',
        output = 'screen',
    )

    rqt_plot_node = Node(
        package = 'rqt_plot',
        executable = 'rqt_plot',
        output = 'screen',
    )

    l_d = LaunchDescription([signal_generator_node, signal_reconstruction_node,
                             rqt_graph_node, rqt_plot_node])
    return l_d