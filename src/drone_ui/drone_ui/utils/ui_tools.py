
import pygame
import rclpy
from rclpy.node import Node
def world_to_screen(world_pos, camera_x, camera_y, zoom_factor,):
    x = (world_pos[0] + camera_x) * zoom_factor
    y = (world_pos[1] + camera_y) * zoom_factor
    
    return (x, y)

# def draw_objects(pg_node, zoom_factor):

#     # Draw friendly drones
#     for fr_obj, fr_drone_pos in pg_node.friendly_drones_positions.items():
#         fr_screen_pos = world_to_screen(fr_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
#         color = (130, 130, 255) if fr_obj in pg_node.selected_drones else (0, 0, 255)
#         pygame.draw.circle(pg_node.screen, color, fr_screen_pos, 10)

#     # Draw enemy drones
#     for en_obj, en_drone_pos in pg_node.enemy_drones_positions.items():
#         en_screen_pos = world_to_screen(en_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
#         pygame.draw.circle(pg_node.screen, (255, 0, 0), en_screen_pos, 10)

def draw_objects(pg_node, zoom_factor):
    """
    Draw friendly and enemy drones, with namespaces visible.
    :param pg_node: The pygame node instance.
    :param zoom_factor: Current zoom factor for scaling.
    """
    # Create a bold font
    font = pygame.font.Font(None, 12)  # Default font, size 24
    font_bold = pygame.font.Font(None, 12)
    font_bold.set_bold(True)

    # Draw friendly drones
    for fr_obj, fr_drone_pos in pg_node.friendly_drones_positions.items():
        fr_screen_pos = world_to_screen(fr_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
        color = (130, 130, 255) if fr_obj in pg_node.selected_drones else (0, 0, 255)
        pygame.draw.circle(pg_node.screen, color, fr_screen_pos, 7)

        # Render the namespace as text
        namespace_surface = font_bold.render(fr_obj, True, (0, 0, 0))  # Black text
        pg_node.screen.blit(namespace_surface, (fr_screen_pos[0] + 12, fr_screen_pos[1] - 12))  # Offset text slightly

    # Draw enemy drones
    for en_obj, en_drone_pos in pg_node.enemy_drones_positions.items():
        en_screen_pos = world_to_screen(en_drone_pos["ui"], pg_node.camera_x, pg_node.camera_y, zoom_factor)
        pygame.draw.circle(pg_node.screen, (255, 0, 0), en_screen_pos, 7)

        # Render the namespace as text
        namespace_surface = font_bold.render(en_obj, True, (0, 0, 0))  # Black text
        pg_node.screen.blit(namespace_surface, (en_screen_pos[0] + 12, en_screen_pos[1] - 12))  # Offset text slightly

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

menu_options = [
    "go_to_goal",
    "high_altitude",
    "low_altitude",
    "attack"
]

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


def handle_menu_selection(pg_node, option):
    """
    Perform actions based on the selected context menu option.
    """
    print(option)
    if option == "go_to_goal":
        print(f"go to goal for: {pg_node.selected_drones}")
        #loop thru selected drones
        # Add logic to set altitude here
    elif option == "high_altitude":
        print(f"high altitude for: {pg_node.selected_drones}")
        # Add logic to send drones to goal
    elif option == "low_altitude":
        print(f"low altitude for: {pg_node.selected_drones}")
        # Add logic to send drones to goal
    elif option == "attack":
        print(f"attack command for: {pg_node.selected_drones}")
        # Add logic to attack target  

def draw_grid(pg_node, zoom_factor, grid_size, screen_height, screen_width):
    scaled_grid_size = grid_size * zoom_factor
    start_x = int((-pg_node.camera_x * zoom_factor) % scaled_grid_size)
    start_y = int((-pg_node.camera_y * zoom_factor) % scaled_grid_size)

    for x in range(start_x, screen_width, int(scaled_grid_size)):
        pygame.draw.line(pg_node.screen, (0, 0, 0), (x, 0), (x, screen_height))
    for y in range(start_y, screen_height, int(scaled_grid_size)):
        pygame.draw.line(pg_node.screen, (0, 0, 0), (0, y), (screen_width, y))
