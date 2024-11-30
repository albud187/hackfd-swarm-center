
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped


def coordinated_movement_goals(pg_node, goal_pose):
    all_friendly_positions = pg_node.friendly_drones_positions
    if len(pg_node.selected_drones) > 1:
        

        centroid_x = 0
        centroid_y = 0
        centroid_z = 0

        # Get centroid of drones current_positions
        for r in pg_node.selected_drones:
            centroid_x += all_friendly_positions[r]["sim"][0]
            centroid_y += all_friendly_positions[r]["sim"][1]
            centroid_z += all_friendly_positions[r]["sim"][2]

        centroid_x = centroid_x / len(pg_node.selected_drones)
        centroid_y = centroid_y / len(pg_node.selected_drones)
        centroid_z = centroid_z / len(pg_node.selected_drones)

        d_pos_fr = {}

        # Get displacements between centroid and current drone positions
        for r in pg_node.selected_drones:
            dx = all_friendly_positions[r]["sim"][0] - centroid_x
            dy = all_friendly_positions[r]["sim"][1] - centroid_y
            dz = all_friendly_positions[r]["sim"][2] - centroid_z

            d_pos_fr[r] = (dx, dy, dz)

        goal_poses = {}
        for r in pg_node.selected_drones:
            gp_x = goal_pose[0] + d_pos_fr[r][0]
            gp_y = goal_pose[1] + d_pos_fr[r][1]
            gp_z = centroid_z

            drone_goal = PoseStamped()
            drone_goal.header.frame_id = "1"
            drone_goal.pose.position.x = gp_x
            drone_goal.pose.position.y = gp_y
            drone_goal.pose.position.z = gp_z + 1

            goal_poses[r] = drone_goal

    else:  # Case for a single drone
        goal_poses = {}
        for r in pg_node.selected_drones:
            drone_goal = PoseStamped()
            drone_goal.header.frame_id = "1"
            drone_goal.pose.position.x = goal_pose[0]
            drone_goal.pose.position.y = goal_pose[1]
            drone_goal.pose.position.z = all_friendly_positions[r]["sim"][2] + 1

            goal_poses[r] = drone_goal

    return goal_poses
