import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_interface.msg import Detections
# import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# please write a py file that sub to a ros2 topic named /align_depth
# which is a standard ros topic that contains depth image.
# please sub the topic, display the dpeth image and save it to current directory
# please save the raw image and display color randered image
class DepthImageSubscriber(Node):

  def __init__(self):
    super().__init__('depth_image_subscriber')
    # self.depth_subscription = self.create_subscription(
    #   Image,
    #   '/camera/camera/aligned_depth_to_color/image_raw',
    #   self.depth_listener_callback,
    #   10)
#    self.color_subscription = self.create_subscription(
#      Image,
#      '/zed/zed_node/left/image_rect_color',
#      self.color_listener_callback,
#      3)
    self.vision_subscription = self.create_subscription(
      Detections,
      '/booster_vision/detection',
      self.vision_listener_callback,
      3)
    self.bridge = CvBridge()
    
    # Create a directory named after the program creation time
    self.save_dir = os.path.join(os.getcwd(), f'images_{self.get_clock().now().to_msg().sec}')
    # os.makedirs(self.save_dir, exist_ok=True)
    self.received_time_history = deque(maxlen=100)
    self.msg_time_history = deque(maxlen=100)
    self.last_received_time = 0
    self.last_msg_time = 0

    # Initialize matplotlib figure
    plt.ion()
    self.fig, self.ax = plt.subplots()
    self.line, = self.ax.plot([], [], label="Timediff (ms)")
    self.ax.set_title("Timediff History")
    self.ax.set_xlabel("Sample")
    self.ax.set_ylabel("Timediff (ms)")
    self.ax.legend()

  def depth_listener_callback(self, msg):
    self.get_logger().info('Receiving depth image')
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    # Convert depth image from uint16 to meters
    depth_image_meters = cv_image * 0.001
    
    # Save the raw depth image with timestamp
    timestamp = self.get_clock().now().to_msg().sec
    raw_image_path = os.path.join(self.save_dir, f'depth_image_raw_{timestamp}.png')
    # cv2.imwrite(raw_image_path, cv_image)
    
#    # Normalize the depth image for display
#    depth_image_normalized = cv2.normalize(depth_image_meters, None, 0, 255, cv2.NORM_MINMAX)
#    depth_image_normalized = np.clip(depth_image_normalized, 0, 255).astype(np.uint8)
#    
#    # Apply color map to the normalized depth image
#    depth_colormap = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)
#    
#    # Save the color rendered depth image
#    color_image_path = os.path.join(self.save_dir, f'depth_image_color_{timestamp}.png')
#    cv2.imwrite(color_image_path, depth_colormap)
    
    # Display the color rendered depth image
    # cv2.imshow('Depth Image', depth_colormap)
    # cv2.waitKey(1)

  def color_listener_callback(self, msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    msg_time_stamp = msg.header.stamp
    msg_time = msg_time_stamp.sec + msg_time_stamp.nanosec / 1e9

    # Save the color image with timestamp
    current_time_stamp = self.get_clock().now().to_msg()
    current_time = current_time_stamp.sec + current_time_stamp.nanosec / 1e9

    timediff = (current_time - msg_time) * 1000
    received_time_interval = abs(current_time - self.last_received_time)
    msg_time_interval = abs(msg_time - self.last_msg_time)

    # import ipdb; ipdb.set_trace()
    if received_time_interval > 1e5:
      received_fps = 0.0
    else:
      self.received_time_history.append(received_time_interval)
      received_fps = 1.0 / np.mean(self.received_time_history)
    
    if msg_time_interval > 1e5:
      msg_fps = 0.0
    else:
      self.msg_time_history.append(msg_time_interval)
      msg_fps = 1.0 / np.mean(self.msg_time_history)
    
    self.get_logger().info(f"latency: {timediff:.2f} ms, received fps: {received_fps:.2f}, msg fps: {msg_fps:.2f}")
    self.last_received_time = current_time
    self.last_msg_time = msg_time
    
  def vision_listener_callback(self, msg):
    msg_time_stamp = msg.header.stamp
    msg_time = msg_time_stamp.sec + msg_time_stamp.nanosec / 1e9

    # Save the color image with timestamp
    current_time_stamp = self.get_clock().now().to_msg()
    current_time = current_time_stamp.sec + current_time_stamp.nanosec / 1e9

    timediff = (current_time - msg_time) * 1000
    received_time_interval = abs(current_time - self.last_received_time)
    msg_time_interval = abs(msg_time - self.last_msg_time)

    # import ipdb; ipdb.set_trace()
    if received_time_interval > 1e5:
      received_fps = 0.0
    else:
      self.received_time_history.append(received_time_interval)
      received_fps = 1.0 / np.mean(self.received_time_history)
    
    if msg_time_interval > 1e5:
      msg_fps = 0.0
    else:
      self.msg_time_history.append(msg_time_interval)
      msg_fps = 1.0 / np.mean(self.msg_time_history)
    
    self.get_logger().info(f"latency: {timediff:.2f} ms, received fps: {received_fps:.2f}, msg fps: {msg_fps:.2f}")
    self.last_received_time = current_time
    self.last_msg_time = msg_time

#    with open(os.path.join(self.save_dir, 'timediff.txt'), 'a') as f:
#      f.write(f"{timediff:.2f},{msg_fps:.2}\n")

    # get current time
    # color_image_path = os.path.join(self.save_dir, f'color_image_{timestamp}.png')
    # cv2.imwrite(color_image_path, cv_image)
    
    # Display the color image
    # cv2.imshow('Color Image', cv_image)
    # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = DepthImageSubscriber()
    rclpy.spin(depth_image_subscriber)
    depth_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
