import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32, Float32MultiArray
import math
import numpy as np


class FormationControllerNode(Node):

    def __init__(self):
        super().__init__('formation_controller')

        # Get the necessary parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('blue_boat_name', 'RAS_TN_DB'),
                ('orange_boat_name', 'RAS_TN_OR'),
                ('desired_distance', 2.0),  # Desired distance between boats
                ('kp_heading', 0.5),  # Heading control proportional gain
                ('kp_velocity', 0.2)  # Velocity control proportional gain
            ]
        )

        # Set up subscribers
        self.blue_pose_sub = self.create_subscription(
            TransformStamped,
            f"/{self.get_parameter('blue_boat_name').value}/pose",
            self.blue_pose_callback,
            10
        )
        self.orange_pose_sub = self.create_subscription(
            TransformStamped,
            f"/{self.get_parameter('orange_boat_name').value}/pose",
            self.orange_pose_callback,
            10
        )

        # Set up publishers
        self.heading_ref_pub = self.create_publisher(
            Float32,
            f"/{self.get_parameter('blue_boat_name').value}/reference/heading",
            10
        )
        self.velocity_ref_pub = self.create_publisher(
            Float32MultiArray,
            f"/{self.get_parameter('blue_boat_name').value}/reference/velocity",
            10
        )

        # Initialize pose variables
        self.blue_pose = None
        self.orange_pose = None

    def blue_pose_callback(self, msg):
        self.blue_pose = msg.transform

        # Only calculate references if both poses are available
        if self.orange_pose is not None:
            self.calculate_and_publish_references()

    def orange_pose_callback(self, msg):
        self.orange_pose = msg.transform

        # Only calculate references if both poses are available
        if self.blue_pose is not None:
            self.calculate_and_publish_references()

    def calculate_and_publish_references(self):
        # Calculate desired heading based on the difference between the poses
        dx = self.orange_pose.translation.x - self.blue_pose.translation.x
        dy = self.orange_pose.translation.y - self.blue_pose.translation.y
        desired_heading = np.arctan2(dy, dx)

        # Calculate distance error
        distance_error = math.sqrt(dx**2 + dy**2) - self.get_parameter('desired_distance').value

        # Calculate velocity reference based on distance error
        velocity_ref = self.get_parameter('kp_velocity').value * distance_error

        # Publish references
        self.heading_ref_pub.publish(Float32(data=desired_heading))
        self.velocity_ref_pub.publish(Float32MultiArray(data=[velocity_ref,0.0,0.0]))

        # In blue_pose_callback and orange_pose_callback:
        self.get_logger().info(f"Blue pose received: {self.blue_pose}")
        self.get_logger().info(f"Orange pose received: {self.orange_pose}")

# In calculate_and_publish_references:
        self.get_logger().info(f"Desired heading: {desired_heading}, Velocity reference: {velocity_ref}")

def main(args=None):
    rclpy.init(args=args)

    formation_controller = FormationControllerNode()

    rclpy.spin(formation_controller)

    formation_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
