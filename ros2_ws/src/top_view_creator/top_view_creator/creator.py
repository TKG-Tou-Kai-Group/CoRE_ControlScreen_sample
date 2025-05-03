import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from numba import jit

#!/usr/bin/env python3

@jit('uint8[:,:,:](uint8[:,:,:],float64[:,:],uint8[:,:,:],float64[:,:])', nopython=True)
def create_top_view(camera1_image, camera1_weight, camera2_image, camera2_weight):
    """Create a top view image by blending two camera images."""
    # Get the dimensions of the images
    height, width = camera1_image.shape[:2]
    
    # Create an empty top view image
    top_view = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Blend the two images using the weights
    for i in range(height):
        for j in range(width):
            top_view[i, j] = (camera1_image[i, j] * camera1_weight[i, j] + 
                              camera2_image[i, j] * camera2_weight[i, j])
    
    return top_view

class TopViewCreator(Node):
    """ROS 2 node to create a top view from a wide-angle camera."""
    
    def __init__(self):
        super().__init__('top_view_creator')
        
        # Declare parameters
        self.declare_parameter('camera_height1', 1.0)  # Height in meters
        self.declare_parameter('camera_height2', 1.0)
        self.declare_parameter('camera_x1', 0.0)       # X position in meters
        self.declare_parameter('camera_x2', 0.0)
        self.declare_parameter('camera_y1', 0.0)       # Y position in meters
        self.declare_parameter('camera_y2', 0.0)
        self.declare_parameter('camera_angle1', 45.0)   # Angle in degrees
        self.declare_parameter('camera_angle2', -135.0)
        self.declare_parameter('fov_degrees', 180.0)  # Field of view in degrees
        self.declare_parameter('output_width', 300)   # Output image width
        self.declare_parameter('output_height', 300)  # Output image height
        self.declare_parameter('ground_width', 4.0)   # Width of ground plane in meters
        self.declare_parameter('ground_height', 4.0)  # Height of ground plane in meters
        
        # Get parameters
        self.camera_height1 = self.get_parameter('camera_height1').value
        self.camera_height2 = self.get_parameter('camera_height2').value
        self.camera_x1 = self.get_parameter('camera_x1').value
        self.camera_x2 = self.get_parameter('camera_x2').value
        self.camera_y1 = self.get_parameter('camera_y1').value
        self.camera_y2 = self.get_parameter('camera_y2').value
        self.camera_angle1 = math.radians(self.get_parameter('camera_angle1').value)  # Convert to radians
        self.camera_angle2 = math.radians(self.get_parameter('camera_angle2').value)
        self.fov_degrees = self.get_parameter('fov_degrees').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        self.ground_width = self.get_parameter('ground_width').value
        self.ground_height = self.get_parameter('ground_height').value

        self.camera1_map_x = None
        self.camera1_map_y = None
        self.camera2_map_x = None
        self.camera2_map_y = None

        self.camera1_weight, self.camera2_weight = self.create_weight_mapping_matrix(self.camera_x1, self.camera_y1, self.camera_x2, self.camera_y2)

        # Calculate derived parameters
        self.fov_radians = math.radians(self.fov_degrees)
        
        # Create CV bridge
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Create subscribers and publishers
        self.subscription = self.create_subscription(
            Image,
            'image1',
            self.image1_callback,
            sensor_qos)
        self.subscription2 = self.create_subscription(
            Image,
            'image2',
            self.image2_callback,
            sensor_qos)
        self.top_view1 = None
        self.top_view2 = None
        self.publisher = self.create_publisher(
            Image,
            'top_view/image',
            10)
        
        self.get_logger().info('TopViewCreator initialized with FOV=%f degrees' % self.fov_degrees)
    
    def create_mapping_matrix(self, image_width, image_height, camera_height, camera_x, camera_y, camera_angle):
        """Create mapping matrices for the perspective transformation."""
        # Create destination coordinates (output image)
        x_out = np.linspace(0, self.ground_width, self.output_width)
        y_out = np.linspace(0, self.ground_height, self.output_height)
        x_grid, y_grid = np.meshgrid(x_out, y_out)
        
        # Center the grid
        x_grid = x_grid - self.ground_width / 2 - camera_x
        y_grid = y_grid - self.ground_height / 2 - camera_y
        
        # Calculate distance from center
        distance = np.sqrt(x_grid**2 + y_grid**2)
        
        # Calculate angles
        theta = np.arctan2(distance, camera_height)
        phi = np.arctan2(y_grid, x_grid) + camera_angle
        
        # Calculate image coordinates
        r_norm = (2 * theta / self.fov_radians)
        max_image_radius = min(image_width, image_height) / 2
        
        x_in = r_norm * max_image_radius * np.cos(phi) + image_width / 2
        y_in = r_norm * max_image_radius * np.sin(phi) + image_height / 2
        
        # Ensure coordinates are within bounds
        x_in = np.clip(x_in, 0, image_width - 1)
        y_in = np.clip(y_in, 0, image_height - 1)
        
        return x_in.astype(np.float32), y_in.astype(np.float32)
    
    def create_weight_mapping_matrix(self, camera1_x, camera1_y, camera2_x, camera2_y):
        """Create a weight mapping matrix for blending images."""
        # Create destination coordinates (output image)
        image_resolution = self.ground_width / self.output_width
        width_coefficient = 2.0 / (self.output_width / 20.0) / image_resolution
        x_out = np.linspace(0, self.ground_width, self.output_width)
        y_out = np.linspace(0, self.ground_height, self.output_height)
        x_grid, y_grid = np.meshgrid(x_out, y_out)
        
        # Center the grid
        x_grid = x_grid - self.ground_width / 2
        y_grid = y_grid - self.ground_height / 2
        
        # Calculate distances to cameras
        distance_to_camera1 = np.sqrt((x_grid - camera1_x)**2 + (y_grid - camera1_y)**2)
        distance_to_camera2 = np.sqrt((x_grid - camera2_x)**2 + (y_grid - camera2_y)**2)
        
        # Calculate weights
        diff_distance = distance_to_camera1 - distance_to_camera2
        camera1_weight = 1.0 / (1.0 + np.exp(-width_coefficient * diff_distance))
        camera2_weight = 1.0 - camera1_weight
        
        return camera1_weight.astype(np.float64), camera2_weight.astype(np.float64)
    
    def image1_callback(self, msg):
        """Process incoming image and publish top view."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Create the mapping matrices
            if self.camera1_map_x is None or self.camera1_map_y is None:
                self.camera1_map_x, self.camera1_map_y = self.create_mapping_matrix(width, height, self.camera_height1, self.camera_x1, self.camera_y1, self.camera_angle1)
            
            # Apply the transformation
            self.top_view1 = cv2.remap(cv_image, self.camera1_map_x, self.camera1_map_y, cv2.INTER_LINEAR)

            if self.top_view2 is not None:
                # Combine the two top views
                top_view = create_top_view(self.top_view1, self.camera1_weight, self.top_view2, self.camera2_weight)
            
                # Convert back to ROS Image message and publish
                top_view_msg = self.bridge.cv2_to_imgmsg(top_view, encoding='bgr8')
                top_view_msg.header = msg.header  # Copy the header
            
                self.publisher.publish(top_view_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def image2_callback(self, msg):
        """Process incoming image and publish top view."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Create the mapping matrices
            if self.camera2_map_x is None or self.camera2_map_y is None:
                self.camera2_map_x, self.camera2_map_y = self.create_mapping_matrix(width, height, self.camera_height2, self.camera_x2, self.camera_y2, self.camera_angle2)
            
            # Apply the transformation
            self.top_view2 = cv2.remap(cv_image, self.camera2_map_x, self.camera2_map_y, cv2.INTER_LINEAR)
                        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    top_view_creator = TopViewCreator()
    
    rclpy.spin(top_view_creator)
    
    top_view_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()