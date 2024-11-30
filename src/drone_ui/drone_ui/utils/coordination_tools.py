
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from scipy.optimize import linear_sum_assignment
import numpy as np


def coordinated_movement_goals(pg_node, goal_pose):
    all_friendly_positions = pg_node.friendly_drones_positions
    selected_drones = pg_node.selected_drones

    if len(selected_drones) > 1:
        # Extract positions of selected drones
        positions = np.array([all_friendly_positions[r]["sim"] for r in selected_drones])

        # Calculate centroid of the selected drones
        centroid = np.mean(positions, axis=0)

        # Calculate displacements from centroid
        displacements = positions - centroid

        # Generate goal poses
        goal_poses = {}
        for r, displacement in zip(selected_drones, displacements):
            goal_x, goal_y = goal_pose[:2] + displacement[:2]
            goal_z = centroid[2]  # Maintain centroid altitude

            drone_goal = PoseStamped()
            drone_goal.header.frame_id = "1"
            drone_goal.pose.position.x = goal_x
            drone_goal.pose.position.y = goal_y
            drone_goal.pose.position.z = goal_z

            goal_poses[r] = drone_goal

    else:  # Case for a single drone
        goal_poses = {}
        for r in selected_drones:
            drone_goal = PoseStamped()
            drone_goal.header.frame_id = "1"
            drone_goal.pose.position.x = goal_pose[0]
            drone_goal.pose.position.y = goal_pose[1]
            drone_goal.pose.position.z = all_friendly_positions[r]["sim"][2]

            goal_poses[r] = drone_goal

    return goal_poses

def set_hover_height(pg_node, hover_height):
    all_friendly_positions = pg_node.friendly_drones_positions
   
    goal_poses = {}
    for r in pg_node.selected_drones:
        drone_goal = PoseStamped()
        drone_goal.header.frame_id = "1"
        drone_goal.pose.position.x = all_friendly_positions[r]["sim"][0]
        drone_goal.pose.position.y = all_friendly_positions[r]["sim"][1]
        drone_goal.pose.position.z = hover_height
        goal_poses[r] = drone_goal

    return goal_poses

def coordinated_attack(pg_node):
    attackers = pg_node.selected_drones
    targets = pg_node.locked_targets


    num_attackers = len(attackers)
    num_targets = len(targets)

    cost_matrix = np.zeros((num_attackers, num_targets))

    attacker_positions = {}
    target_positions = {}
    #populate cost matrix with distances
    pass