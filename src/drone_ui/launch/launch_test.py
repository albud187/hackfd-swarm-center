from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'launch', 'drone_action_model', 'launch_drones.py'],
            output='screen'
        ),

        Node(
            package='drone_ui',
            executable='pygame_node',
            name='pygame_node'
        ),
        

    ])
