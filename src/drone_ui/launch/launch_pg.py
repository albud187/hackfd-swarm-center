from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch the Pygame demo node
        Node(
            package='drone_ui',
            executable='pygame_node',
            name='pygame_node'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
