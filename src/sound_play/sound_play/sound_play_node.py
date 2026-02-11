import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# import pygame
from os import path
from ament_index_python.packages import get_package_share_directory
import subprocess

class SoundPlayNode(Node):
    def __init__(self):
        super().__init__('sound_play_node')
        self.sub_play = self.create_subscription(
            String,
            'play_sound',
            self.callback_play,
            10)
        self.sub_play  # prevent unused variable warning

        self.sub_speak = self.create_subscription(
            String,
            'speak',
            self.callback_speak,
            10)
        self.sub_speak  # prevent unused variable warning

        # 初始化pygame mixer
        # pygame.mixer.init()

    def callback_play(self, msg):
        self.get_logger().info('Received sound command: "%s"' % msg.data)
        share_dir = get_package_share_directory("sound_play")
        sound_file_path = path.join(share_dir, 'sounds', msg.data + '.mp3')
        if path.exists(sound_file_path):
            # 使用 pygame
            # sound = pygame.mixer.Sound(sound_file_path)
            # sound.play()

            # 使用 mpg321
            subprocess.run(['mpg321', sound_file_path])
        else:
            print('sound file not found: ' + sound_file_path)
    
    def callback_speak(self, msg):
        print('Received speak command: "%s"' % msg.data)
        subprocess.run(['espeak', msg.data])

def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
