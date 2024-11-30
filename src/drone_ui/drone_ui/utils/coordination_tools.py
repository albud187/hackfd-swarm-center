
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from scipy.optimize import linear_sum_assignment
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

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
    """
    Assign attackers (selected drones) to targets using linear sum assignment.
    """
    attackers = pg_node.selected_drones
    targets = pg_node.locked_targets  # Assuming targets have been pre-selected

    if not attackers or not targets:
        print("No attackers or targets available for assignment.")
        return {}

    # Extract positions of attackers and targets
    attacker_positions = np.array([pg_node.friendly_drones_positions[attacker]["sim"][:2] for attacker in attackers])
    target_positions = np.array([pg_node.enemy_drones_positions[target]["sim"][:2] for target in targets])

    # Calculate cost matrix based on Euclidean distance
    cost_matrix = np.linalg.norm(attacker_positions[:, None, :] - target_positions[None, :, :], axis=2)

    # Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost_matrix)

    # Create assignment dictionary
    assignments = {}
    for attacker_idx, target_idx in zip(row_indices, col_indices):
        attacker_ns = attackers[attacker_idx]
        target_ns = targets[target_idx]

        # Create PoseStamped for the target
        target_pose = PoseStamped()
        target_pose.header.frame_id = "2"
        target_pose.pose.position.x = pg_node.enemy_drones_positions[target_ns]["sim"][0]
        target_pose.pose.position.y = pg_node.enemy_drones_positions[target_ns]["sim"][1]
        target_pose.pose.position.z = pg_node.enemy_drones_positions[target_ns]["sim"][2]

        assignments[attacker_ns] = target_pose

    return assignments