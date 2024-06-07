import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('minichallenge3'),
            'config',
            'params.yaml'
    )

    generation_node = Node(
        package= 'minichallenge3',
        executable= 'controller',
        output = 'screen',
        parameters= [{'use_sim_time':True}, {config}]
    )

    ld = LaunchDescription([generation_node])
    return ld