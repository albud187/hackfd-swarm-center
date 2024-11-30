import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose
from drone_ui.utils._constants import *
from drone_ui.utils.topic_tools import (get_topic_list, filter_topics)
import pprint
grid_size = 50

zoom_factor = 1.0
FPS = 60
screen_width = 800
screen_height = 600

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

        self.game_object = {'pos': (0, 0), 'radius': 20}

        self.topic_list = get_topic_list()
        print("XXXXXX")
        print("XXXXXX")
        print("XXXXXX")

        pprint.pprint(self.topic_list)
        print("XXXXXX")
        print("XXXXXX")
        print("XXXXXX")
        self.friendly_drones = {}
        self.friendly_pose_subs = {}

        self.enemy_drones = {}
        self.enemy_pose_subs = {}

    def pose_cb(self, msg, ns):
        if ns[0]=="r":
            self.friendly_drones[ns] = (msg.position.x, msg.position.y)
        if ns[0] == "t":
            self.enemy_drones[ns] = (msg.position.x, msg.positiony)

    def update_pose_subs(self):
        """
        inputs: str fr_ns, str en_ns
        outputs: none
        description: updates the pose subscribers, adding more as more drones are spawned in the simulation
        updates both friendly drones and enemy drones based on namespace prefix
        """

        




        pass

    def drone_pose_cb(self, msg):
        self.game_object['pos'] = (grid_size*msg.position.x, -grid_size*msg.position.y)

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

        #draw empty game world with white square and grid
        self.screen.fill((255, 255, 255))

        #draw grid
        # Draw grid
        scaled_grid_size = grid_size * zoom_factor
        start_x = int((-self.camera_x * zoom_factor) % scaled_grid_size)
        start_y = int((-self.camera_y * zoom_factor) % scaled_grid_size)

        for x in range(start_x, screen_width, int(scaled_grid_size)):
            pygame.draw.line(self.screen, (0, 0, 0), (x, 0), (x, screen_height))
        for y in range(start_y, screen_height, int(scaled_grid_size)):
            pygame.draw.line(self.screen, (0, 0, 0), (0, y), (screen_width, y))

        obj_screen_pos = self.world_to_screen(self.game_object['pos'])
        pygame.draw.circle(self.screen, (255,0,0), obj_screen_pos, self.game_object['radius'])

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