import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from drone_ui.utils._constants import *
from drone_ui.utils.topic_tools import (get_topic_list, filter_topics, get_ns)
import pprint
import time
grid_size = 50

zoom_factor = 1.0
FPS = 60
screen_width = 800
screen_height = 600

# Variables for selection rectangle
is_drawing_rect = False
rect_start_pos = (0, 0)
rect_end_pos = (0, 0)
selected_drones = []

# Variables for context menu
context_menu_visible = False
context_menu_pos = (0, 0)
context_menu_options = ['Option 1', 'Option 2']


# Variables for panning
is_panning = False
pan_start_pos = (0, 0)

class PygameNode(Node):

    def __init__(self):

        super().__init__('pygame_node')

        self.camera_x = screen_width/2
        self.camera_y = screen_height/2

        pygame.init()
        pygame.display.set_caption("pygame_node")
        self.FramePerSec = pygame.time.Clock()
        self.FPS = FPS
        self.screen = pygame.display.set_mode((screen_width, screen_height))

        #self.game_object = {'pos': (0, 0), 'radius': 20}

        #get all topics
        time.sleep(2)
        self.topic_list = get_topic_list()

        ##get pose topic for subscribers
        self.friendly_pose_topics = filter_topics(self.topic_list, "/r", "RPY_pose")
        self.enemy_pose_topics = filter_topics(self.topic_list, "/t", "RPY_pose")
        
        #init friendly positions as 0,0
        self.friendly_drones_positions = {}
        self.goal_pose_pubs = {}
        for tfd in self.friendly_pose_topics:
            f_ns = get_ns(tfd)
            self.friendly_drones_positions[f_ns] = (0,0)

        #init enemy positions as 0,0
        self.enemy_drones_positions = {}
        for te in self.enemy_pose_topics:
            e_ns = get_ns(te)
            self.enemy_drones_positions[e_ns] = (0,0)

        self.friendly_pose_subs = {}
        self.enemy_pose_subs = {}
        self.update_pose_subs()

        ##genereate pose topic publishers
        self.goal_pose_topics = filter_topics(self.topic_list, "/r", "goal_pose")

        self.goal_pose_pubs = {}
        for gp in self.goal_pose_topics:
            f_ns_gp = get_ns(gp)
            self.goal_pose_pubs[f_ns_gp] = self.create_publisher(PoseStamped, gp, 10)
        
        print(self.goal_pose_pubs)
        
    def fr_pose_cb(self, msg, f_ns):
        """
        Callback for friendly drone pose updates.
        """
        self.friendly_drones_positions[f_ns] = (grid_size * msg.pose.position.x, -grid_size * msg.pose.position.y)

    def en_pose_cb(self, msg, e_ns):
        """
        Callback for enemy drone pose updates.
        """
        self.enemy_drones_positions[e_ns] = (grid_size * msg.pose.position.x, -grid_size * msg.pose.position.y)

    def update_pose_subs(self):
        """
        Dynamically updates the pose subscribers for friendly and enemy drones.
        """
        for tfd in self.friendly_pose_topics:
        
            f_ns = get_ns(tfd)
            if f_ns not in self.friendly_pose_subs:
                # Use default arguments in the lambda to correctly bind the namespace
                self.friendly_pose_subs[f_ns] = self.create_subscription(
                    PoseStamped,
                    tfd,
                    lambda msg, ns=f_ns: self.fr_pose_cb(msg, ns),
                    20
                )
        
        for ted in self.enemy_pose_topics:
           
            e_ns = get_ns(ted)
            if e_ns not in self.friendly_pose_subs:
                # Use default arguments in the lambda to correctly bind the namespace
                self.enemy_pose_subs[e_ns] = self.create_subscription(
                    PoseStamped,
                    ted,
                    lambda msg, ns=e_ns: self.en_pose_cb(msg, ns),
                    20
                )

    def world_to_screen(self, world_pos):
        x = (world_pos[0] + self.camera_x) * zoom_factor
        y = (world_pos[1] + self.camera_y) * zoom_factor
        
        return (x, y)
   
    def render_loop(self):
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        while rclpy.ok():
            self.render()
            self.FramePerSec.tick(self.FPS)
            executor.spin_once(timeout_sec=0.1)

    def close(self):
        pygame.quit()

    def render(self):
        global is_drawing_rect, rect_start_pos, rect_end_pos, selected_drones

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.display.quit()
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()
                quit()

            elif event.type == pygame.KEYDOWN:
                # Panning the view with arrow keys
                if event.key == pygame.K_LEFT:
                    self.camera_x += 20 / zoom_factor
                elif event.key == pygame.K_RIGHT:
                    self.camera_x -= 20 / zoom_factor
                elif event.key == pygame.K_UP:
                    self.camera_y += 20 / zoom_factor
                elif event.key == pygame.K_DOWN:
                    self.camera_y -= 20 / zoom_factor

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:  # Left mouse button
                is_drawing_rect = True
                rect_start_pos = pygame.mouse.get_pos()
                rect_end_pos = rect_start_pos

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:  # Left mouse button
                is_drawing_rect = False
                rect_end_pos = pygame.mouse.get_pos()
                selected_drones = self.get_selected_drones(rect_start_pos, rect_end_pos)

            elif event.type == pygame.MOUSEMOTION:
                if is_drawing_rect:
                    rect_end_pos = pygame.mouse.get_pos()

        # Draw empty game world with white background and grid
        self.screen.fill((255, 255, 255))

        # Draw grid
        scaled_grid_size = grid_size * zoom_factor
        start_x = int((-self.camera_x * zoom_factor) % scaled_grid_size)
        start_y = int((-self.camera_y * zoom_factor) % scaled_grid_size)

        for x in range(start_x, screen_width, int(scaled_grid_size)):
            pygame.draw.line(self.screen, (0, 0, 0), (x, 0), (x, screen_height))
        for y in range(start_y, screen_height, int(scaled_grid_size)):
            pygame.draw.line(self.screen, (0, 0, 0), (0, y), (screen_width, y))

        # Draw friendly drones
        for fr_obj, fr_drone_pos in self.friendly_drones_positions.items():
            fr_screen_pos = self.world_to_screen((fr_drone_pos[0], fr_drone_pos[1]))
            color = (130, 130, 255) if fr_obj in selected_drones else (0, 0, 255)
            pygame.draw.circle(self.screen, color, fr_screen_pos, 10)

        # Draw enemy drones
        for en_obj, en_drone_pos in self.enemy_drones_positions.items():
            en_screen_pos = self.world_to_screen((en_drone_pos[0], en_drone_pos[1]))
            pygame.draw.circle(self.screen, (255, 0, 0), en_screen_pos, 10)

        # Draw selection rectangle
        if is_drawing_rect:
            rect_x = min(rect_start_pos[0], rect_end_pos[0])
            rect_y = min(rect_start_pos[1], rect_end_pos[1])
            rect_width = abs(rect_end_pos[0] - rect_start_pos[0])
            rect_height = abs(rect_end_pos[1] - rect_start_pos[1])
            pygame.draw.rect(self.screen, (0, 255, 0), (rect_x, rect_y, rect_width, rect_height), 2)

        pygame.display.update()

    def get_selected_drones(self, start_pos, end_pos):
        """
        Returns a list of friendly drones within the selection rectangle.
        """
        x_min = min(start_pos[0], end_pos[0])
        x_max = max(start_pos[0], end_pos[0])
        y_min = min(start_pos[1], end_pos[1])
        y_max = max(start_pos[1], end_pos[1])

        selected = []
        for fr_obj, fr_drone_pos in self.friendly_drones_positions.items():
            fr_screen_pos = self.world_to_screen((fr_drone_pos[0], fr_drone_pos[1]))
            if x_min <= fr_screen_pos[0] <= x_max and y_min <= fr_screen_pos[1] <= y_max:
                selected.append(fr_obj)
        return selected
                    
def main(args=None):
    rclpy.init(args=args)
    pygame_node = PygameNode()
    try:
        pygame_node.render_loop()
    except KeyboardInterrupt:
        pass
    finally:
        pygame_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()