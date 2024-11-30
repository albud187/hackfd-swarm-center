import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import xacro
import csv


USE_SIM_TIME = LaunchConfiguration("use_sim_time", default="true")
USE_GUI = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"], description="Whether to execute gzclient")
PKG_GAZEBO_ROS = get_package_share_directory('gazebo_ros')
XACRO_FILE_NAME = "sjtu_drone_multi.urdf.xacro"
XACRO_FILE_PATH = os.path.join(get_package_share_directory("sjtu_drone_description"),"urdf", XACRO_FILE_NAME)
WORLD_DIR = "/workdir/assets/worlds"
WORLD_FILE = os.path.join(WORLD_DIR, "mission_2.world")

CSV_PATH = "/workdir/data/scenario2.csv"

def read_drones_csv(csv_file):
    """
    Reads a CSV file with columns 'drone_id', 'x_pos', 'y_pos'
    and returns a list of drone IDs and a dictionary of their positions.

    :param csv_file: Path to the CSV file
    :return: tuple (list of drone IDs, dictionary of drone positions)
    """
    drone_ids = []
    drone_positions = {}

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            # print(row)
            # print(row.keys())
            drone_id = row['drone_id']
            x_pos, y_pos = row['x_pos'], row['y_pos']

            drone_ids.append(drone_id)
            drone_positions[drone_id] = x_pos +" " + y_pos + " " + "0.0"

    return drone_ids, drone_positions

def spawn_drone_description(ns, init_pose, use_sim_time):
    r_n_doc = xacro.process_file(XACRO_FILE_PATH, mappings = {"drone_id":ns})
    r_n_desc = r_n_doc.toprettyxml(indent='  ')

    result = [
            Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{'frame_prefix': ns +'/','use_sim_time': use_sim_time, 'robot_description': r_n_desc}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=ns+"_" +'joint_state_publisher',
            namespace=ns,
            output='screen',
        ),
        
        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[r_n_desc, ns, init_pose],
            output="screen"
        )
    ]
    
    return result

R_NS, init_poses = read_drones_csv(CSV_PATH)

def multi_drone_description(R_NS, init_poses):
    result = []
    for ns in R_NS:
        drone_spawn_instructions = spawn_drone_description(ns, init_poses[ns], USE_SIM_TIME)
        result = result + drone_spawn_instructions
    return result


def drone_bringup(ns):

    #add more nodes here
    
    drone_result = [
        Node(
            package="drone_action_model",
            executable="state_reporter_node",
            namespace=ns
        ),

        # Node(
        #     package="drone_action_model",
        #     executable="camera_node",
        #     namespace=ns
        #     )
    ]

    if ns.startswith("r"):
        fr_drone_result = [
            Node(
            package="drone_action_model",
            executable="motion_planner_node",
            namespace=ns
            ),
            Node(
                package="drone_action_model",
                executable="kinematics_node",
                namespace=ns,
            )
        ]

        drone_result = drone_result + fr_drone_result
        
    return drone_result

def multi_drone_bringup(R_NS):
    result = []
    for ns in R_NS:
        drone_brinup_nodes = drone_bringup(ns)
        result = result + drone_brinup_nodes
    return result

def launch_gzclient(context, *args, **kwargs):
    if context.launch_configurations.get('use_gui') == 'true':
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(PKG_GAZEBO_ROS, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'verbose': 'true'}.items()
        )]
    return []

def generate_launch_description():

    LD =[
        USE_GUI,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(PKG_GAZEBO_ROS, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': WORLD_FILE,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient)
    ]

    LD = LD + multi_drone_description(R_NS, init_poses)
    LD = LD + multi_drone_bringup(R_NS)
    return LaunchDescription(LD)