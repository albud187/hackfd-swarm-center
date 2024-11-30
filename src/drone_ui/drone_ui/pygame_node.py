import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from drone_ui.utils._constants import *
from drone_ui.utils.topic_tools import (get_topic_list, filter_topics, get_ns)
from drone_ui.utils.ui_tools import (world_to_screen, draw_objects, get_selected_drones)
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

        self.selected_drones = []

        #get all topics
        time.sleep(2)
        self.topic_list = get_topic_list()

        ##get pose topic for subscribers
        self.friendly_pose_topics = filter_topics(self.topic_list, "/r", "RPY_pose")
        self.enemy_pose_topics = filter_topics(self.topic_list, "/t", "RPY_pose")
        
        self.friendly_drones_positions = {}
        self.enemy_drones_positions = {}

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
        
    def drone_pose_cb(self, msg, ns):
        if ns.startswith("/r"):
            self.friendly_drones_positions[ns] = {"ui":(grid_size * msg.pose.position.x, -grid_size * msg.pose.position.y), 
                                                  "sim": (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)}

        if ns.startswith("/t"):
            self.enemy_drones_positions[ns] = {"ui":(grid_size * msg.pose.position.x, -grid_size * msg.pose.position.y), 
                                                  "sim": (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)}

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
                    lambda msg, ns=f_ns: self.drone_pose_cb(msg, ns),
                    20
                )
        
        for ted in self.enemy_pose_topics:
           
            e_ns = get_ns(ted)
            if e_ns not in self.friendly_pose_subs:
                # Use default arguments in the lambda to correctly bind the namespace
                self.enemy_pose_subs[e_ns] = self.create_subscription(
                    PoseStamped,
                    ted,
                    lambda msg, ns=e_ns: self.drone_pose_cb(msg, ns),
                    20
                )
   
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
        global is_drawing_rect, rect_start_pos, rect_end_pos
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
                self.selected_drones = get_selected_drones(self, rect_start_pos, rect_end_pos, zoom_factor)
                print(self.selected_drones)
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

        draw_objects(self, zoom_factor)
        
        # Draw selection rectangle
        if is_drawing_rect:
            rect_x = min(rect_start_pos[0], rect_end_pos[0])
            rect_y = min(rect_start_pos[1], rect_end_pos[1])
            rect_width = abs(rect_end_pos[0] - rect_start_pos[0])
            rect_height = abs(rect_end_pos[1] - rect_start_pos[1])
            pygame.draw.rect(self.screen, (0, 255, 0), (rect_x, rect_y, rect_width, rect_height), 2)

        pygame.display.update()

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