
import pygame
import rclpy
from rclpy.node import Node
import pprint
from drone_ui.utils.coordination_tools import (
    coordinated_movement_goals,
    set_hover_height,
    coordinated_attack)

def world_to_screen(pg_node, world_pos):
    x = (world_pos[0] + pg_node.camera_x) * pg_node.zoom_factor
    y = (world_pos[1] + pg_node.camera_y) * pg_node.zoom_factor
    
    return (x, y)

#origin markers
def generate_origin_markers(pg_node, max_distance):
    """
    Generate origin markers scaled for the ui grid.
    """
    markers = []
    for i in range(-max_distance, max_distance + 1):
        # Scale simulation coordinates to UI coordinates
        x = pg_node.grid_size * (i / pg_node.grid_scale)
        markers.append((x, 0))  # Horizontal line (x-axis)
        markers.append((0, -x))  # Vertical line (y-axis, inverted for UI)

    return markers

def draw_objects(pg_node):
    """
    Draw friendly and enemy drones, with namespaces visible.
    """
    # Create a bold font
    font = pygame.font.Font(None, 12)  # Default font, size 24
    font_bold = pygame.font.Font(None, 12)
    font_bold.set_bold(True)

    # Draw friendly drones
    for fr_obj, fr_drone_pos in pg_node.friendly_drones_positions.items():
        fr_screen_pos = world_to_screen(pg_node, fr_drone_pos["ui"])
        color = (130, 130, 255) if fr_obj in pg_node.selected_drones else (0, 0, 255)
        pygame.draw.circle(pg_node.screen, color, fr_screen_pos, 7)

        # Render the namespace as text
        namespace_surface = font_bold.render(fr_obj, True, (0, 0, 255))  # blue text
        pg_node.screen.blit(namespace_surface, (fr_screen_pos[0] + 12, fr_screen_pos[1] - 12))  # Offset text slightly

    # Draw enemy drones
    for en_obj, en_drone_pos in pg_node.enemy_drones_positions.items():
        en_screen_pos = world_to_screen(pg_node, en_drone_pos["ui"])
        
        color = (150, 0, 0) if en_obj in pg_node.selected_targets else (255, 0, 0)
        color = (0, 0, 0) if en_obj in pg_node.locked_targets else (255, 0, 0)
        pygame.draw.circle(pg_node.screen, color, en_screen_pos, 7)

        # Render the namespace as text
        namespace_surface = font_bold.render(en_obj, True, (255, 0, 0))  # red text
        pg_node.screen.blit(namespace_surface, (en_screen_pos[0] + 12, en_screen_pos[1] - 12))  # Offset text slightly

    # draw origin dots
     # Draw origin markers
    origin_marker_locations = generate_origin_markers(pg_node, 4)
    for marker in origin_marker_locations:
        marker_pos = world_to_screen(pg_node, marker)
        pygame.draw.circle(pg_node.screen, (0, 0, 0), marker_pos, 3)

def get_selected_objects(pg_node, start_pos, end_pos):
    """
    Returns a list of friendly drones within the selection rectangle.
    """
    x_min = min(start_pos[0], end_pos[0])
    x_max = max(start_pos[0], end_pos[0])
    y_min = min(start_pos[1], end_pos[1])
    y_max = max(start_pos[1], end_pos[1])

    selected_friendly = []
    selected_targets = []
    for fr_obj, fr_drone_pos in pg_node.friendly_drones_positions.items():
        fr_screen_pos = world_to_screen(pg_node, fr_drone_pos["ui"])
        if x_min <= fr_screen_pos[0] <= x_max and y_min <= fr_screen_pos[1] <= y_max:
            selected_friendly.append(fr_obj)

    for tgt_obj, enemy_pos in pg_node.enemy_drones_positions.items():
        en_screen_pos = world_to_screen(pg_node, enemy_pos["ui"])
        if x_min <= en_screen_pos[0] <= x_max and y_min <= en_screen_pos[1] <= y_max:
            selected_targets.append(tgt_obj)

    result_selections = {
        "friendly": selected_friendly,
        "targets":selected_targets
    }
    return result_selections

def draw_menu(screen, menu_options, position, font):
    """
    Draw a context menu on the screen.
    """
    menu_width = 200
    menu_height = len(menu_options) * 30
    menu_rect = pygame.Rect(position[0], position[1], menu_width, menu_height)
    
    # Draw menu background
    pygame.draw.rect(screen, (200, 200, 200), menu_rect)
    pygame.draw.rect(screen, (0, 0, 0), menu_rect, 2)  # Border

    # Draw menu options
    for i, option in enumerate(menu_options):
        text_surface = font.render(option, True, (0, 0, 0))
        screen.blit(text_surface, (position[0] + 10, position[1] + i * 30))
    
    return menu_rect

def mouse_ui_to_sim(pg_node, mousepos):
    mouse_x = mousepos[0]
    mouse_y = mousepos[1]

    x_sim = pg_node.grid_scale*((mouse_x/pg_node.zoom_factor) - pg_node.camera_x)/pg_node.grid_size
    y_sim = -pg_node.grid_scale*((mouse_y/pg_node.zoom_factor) - pg_node.camera_y)/pg_node.grid_size

    sim_pos = (x_sim, y_sim)
    return sim_pos

def handle_menu_selection(pg_node, option):
    """
    Perform actions based on the selected context menu option.
    """
    print(option)
    if option == "go_to_goal":
        print(f"go to goal for: {pg_node.selected_drones}")
        mouse_pos = pygame.mouse.get_pos()
        real_pos = mouse_ui_to_sim(pg_node, mouse_pos)
        fr_goal_poses = coordinated_movement_goals(pg_node, real_pos)
        for r in pg_node.selected_drones:
            pg_node.goal_pose_pubs[r].publish(fr_goal_poses[r])
       
    elif option == "high_hover":
        fr_goal_poses = set_hover_height(pg_node, 10.0)
        for r in pg_node.selected_drones:
            pg_node.goal_pose_pubs[r].publish(fr_goal_poses[r])

    elif option == "low_hover":
        fr_goal_poses = set_hover_height(pg_node, 4.0)
        for r in pg_node.selected_drones:
            pg_node.goal_pose_pubs[r].publish(fr_goal_poses[r])

    elif option == "add_targets":
        pg_node.locked_targets = pg_node.selected_targets

    elif option == "clear_targets":
        pg_node.locked_targets = []
    
    elif option == "attack":
        target_assignment = coordinated_attack(pg_node)
        print(f"attack command for: {pg_node.selected_drones} to attack {pg_node.locked_targets}")

        for r in list(target_assignment.keys()):
            pg_node.goal_pose_pubs[r].publish(target_assignment[r])

    
def draw_grid(pg_node, screen_height, screen_width):
    """
    Draws the grid on the screen with proper scaling.
    """
    scaled_grid_size = pg_node.grid_size * pg_node.zoom_factor
    start_x = int((-pg_node.camera_x * pg_node.zoom_factor) % scaled_grid_size)
    start_y = int((-pg_node.camera_y * pg_node.zoom_factor) % scaled_grid_size)
    grid_label_step = int(pg_node.grid_scale)

    for x in range(start_x, screen_width, int(scaled_grid_size)):
        pygame.draw.line(pg_node.screen, (200, 200, 200), (x, 0), (x, screen_height))
        label_x = (x - start_x) // scaled_grid_size * grid_label_step
        font = pygame.font.SysFont(None, 18)
        # pg_node.screen.blit(label, (x + 2, 2))

    for y in range(start_y, screen_height, int(scaled_grid_size)):
        pygame.draw.line(pg_node.screen, (200, 200, 200), (0, y), (screen_width, y))
        label_y = (y - start_y) // scaled_grid_size * grid_label_step
        font = pygame.font.SysFont(None, 18)
        # pg_node.screen.blit(label, (2, y + 2))