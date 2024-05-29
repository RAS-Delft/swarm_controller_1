import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32, Float32MultiArray
import math
import numpy as np

class SwarmControllerNode(Node):

    def __init__(self):
        super().__init__('swarm_controller')

        # Get the necessary parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('orange_boat', 'RAS_TN_OR'),
                ('dark_blue_boat', 'RAS_TN_DB'),
                ('green_boat', 'RAS_TN_GR'),
                ('desired_distance', 0.005),  # Desired distance between boats
                ('separation_distance', 0.0001),  # Minimum separation distance between boats
                ('kp_heading', 0.0005),  # Heading control proportional gain
                ('kp_velocity', 0.005)  # Velocity control proportional gain
            ]
        )

        # Boat identifiers
        self.boats = [
            self.get_parameter('orange_boat').value,
            self.get_parameter('dark_blue_boat').value,
            self.get_parameter('green_boat').value
        ]

        # Set up subscribers
        self.pose_subscribers = {
            boat: self.create_subscription(
                TransformStamped,
                f"/{boat}/pose",
                lambda msg, boat=boat: self.pose_callback(msg, boat),
                10
            ) for boat in self.boats
        }
        self.heading_subscribers = {
            boat: self.create_subscription(
                Float32,
                f"/{boat}/telemetry/heading",
                lambda msg, boat=boat: self.heading_callback(msg, boat),
                10
            ) for boat in self.boats
        }
        self.velocity_subscribers = {
            boat: self.create_subscription(
                Float32MultiArray,
                f"/{boat}/state/velocity",
                lambda msg, boat=boat: self.velocity_callback(msg, boat),
                10
            ) for boat in self.boats
        }

        # Set up publishers
        self.heading_publishers = {
            boat: self.create_publisher(
                Float32,
                f"/{boat}/reference/heading",
                10
            ) for boat in self.boats
        }
        self.velocity_publishers = {
            boat: self.create_publisher(
                Float32MultiArray,
                f"/{boat}/reference/velocity",
                10
            ) for boat in self.boats
        }

        # Initialize state variables
        self.poses = {boat: None for boat in self.boats}
        self.headings = {boat: None for boat in self.boats}
        self.velocities = {boat: None for boat in self.boats}

    def pose_callback(self, msg, boat):
        self.poses[boat] = msg.transform
        self.check_all_data_received()

    def heading_callback(self, msg, boat):
        self.headings[boat] = msg.data
        self.check_all_data_received()

    def velocity_callback(self, msg, boat):
        self.velocities[boat] = msg.data
        self.check_all_data_received()

    def check_all_data_received(self):
        if all(self.poses.values()) and all(self.headings.values()) and all(self.velocities.values()):
            self.calculate_and_publish_references()

    def calculate_and_publish_references(self):
        desired_distance = self.get_parameter('desired_distance').value
        separation_distance = self.get_parameter('separation_distance').value

        positions = {boat: (pose.translation.x, pose.translation.y) for boat, pose in self.poses.items()}

        # Boids algorithm: Calculate velocity and heading for each boat
        for boat in self.boats:
            position = positions[boat]
            heading = self.headings[boat]
            velocity = self.velocities[boat]

            # Separation
            separation_force = np.array([0.0, 0.0])
            for other_boat in self.boats:
                if other_boat != boat:
                    other_position = positions[other_boat]
                    distance = np.linalg.norm(np.array(position) - np.array(other_position))
                    if distance < separation_distance:
                        separation_force += np.array(position) - np.array(other_position)

            # Alignment
            avg_heading = np.mean([self.headings[other_boat] for other_boat in self.boats if other_boat != boat])

            # Cohesion
            center_of_mass = np.mean([positions[other_boat] for other_boat in self.boats if other_boat != boat], axis=0)
            cohesion_force = center_of_mass - np.array(position)

            # Calculate desired heading and velocity
            desired_heading = np.arctan2(cohesion_force[1], cohesion_force[0]) + avg_heading + separation_force[1]
            distance_error = np.linalg.norm(cohesion_force) - desired_distance
            velocity_ref = self.get_parameter('kp_velocity').value * distance_error

            # Publish references
            self.heading_publishers[boat].publish(Float32(data=desired_heading))
            self.velocity_publishers[boat].publish(Float32MultiArray(data=[velocity_ref, 0.0, 0.0]))

            # Logging
            self.get_logger().info(f"{boat} - Desired heading: {desired_heading}, Velocity reference: {velocity_ref}")

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmControllerNode()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
