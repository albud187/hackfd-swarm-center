
import pygame
def world_to_screen(world_pos, camera_x, camera_y, zoom_factor,):
    x = (world_pos[0] + camera_x) * zoom_factor
    y = (world_pos[1] + camera_y) * zoom_factor
    
    return (x, y)

def draw_objects(pg_node, zoom_factor):

    # Draw friendly drones
    for fr_obj, fr_drone_pos in pg_node.friendly_drones_positions.items():
        fr_screen_pos = world_to_screen(fr_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
        color = (130, 130, 255) if fr_obj in pg_node.selected_drones else (0, 0, 255)
        pygame.draw.circle(pg_node.screen, color, fr_screen_pos, 10)

    # Draw enemy drones
    for en_obj, en_drone_pos in pg_node.enemy_drones_positions.items():
        en_screen_pos = world_to_screen(en_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
        pygame.draw.circle(pg_node.screen, (255, 0, 0), en_screen_pos, 10)

def get_selected_drones(pg_node, start_pos, end_pos, zoom_factor):
    """
    Returns a list of friendly drones within the selection rectangle.
    """
    x_min = min(start_pos[0], end_pos[0])
    x_max = max(start_pos[0], end_pos[0])
    y_min = min(start_pos[1], end_pos[1])
    y_max = max(start_pos[1], end_pos[1])

    selected = []
    for fr_obj, fr_drone_pos in pg_node.friendly_drones_positions.items():
        fr_screen_pos = world_to_screen(fr_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
        if x_min <= fr_screen_pos[0] <= x_max and y_min <= fr_screen_pos[1] <= y_max:
            selected.append(fr_obj)
    return selected