import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32, Float32MultiArray
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
                ('desired_distance', 0.09),  # Desired distance between boats
                ('separation_distance', 0.04),  # Minimum separation distance between boats
                ('kp_heading', 0.03),  # Heading control proportional gain
                ('kp_velocity', 0.005),  # Velocity control proportional gain
                ('max_distance', 0.15)  # Maximum distance from the center of all boats
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
        max_distance = self.get_parameter('max_distance').value

        positions = {boat: (pose.translation.x, pose.translation.y) for boat, pose in self.poses.items()}
        velocities = {boat: (vel[0], vel[1]) for boat, vel in self.velocities.items()}

        center_of_mass = np.mean([np.array(position) for position in positions.values()], axis=0)

        # For each boat
        for boat in self.boats:
            position = positions[boat]

            # Check if boat is outside max distance and prioritize steering back to center
            distance_to_center = np.linalg.norm(np.array(position) - center_of_mass)
            if distance_to_center > max_distance:
                direction_to_center = center_of_mass - np.array(position)
                direction_to_center_normalized = direction_to_center / np.linalg.norm(direction_to_center)
                velocities[boat] = (
                    direction_to_center_normalized[0] * self.get_parameter('kp_heading').value,
                    direction_to_center_normalized[1] * self.get_parameter('kp_heading').value
                )
                self.get_logger().info(f"{boat} is too far from center, prioritizing steering back")

                # Default values when prioritizing returning to center
                xpos_avg, ypos_avg = center_of_mass
            else:
                xpos_avg, ypos_avg, xvel_avg, yvel_avg = 0, 0, 0, 0
                neighboring_boids = 0
                close_dx, close_dy = 0, 0

                # For every other boat
                for other_boat in self.boats:
                    if other_boat != boat:
                        other_position = positions[other_boat]
                        dx = position[0] - other_position[0]
                        dy = position[1] - other_position[1]
                        squared_distance = dx * dx + dy * dy

                        # Check if within separation distance
                        if squared_distance < separation_distance ** 2:
                            close_dx += dx
                            close_dy += dy
                        # Otherwise, consider for alignment and cohesion
                        else:
                            xpos_avg += other_position[0]
                            ypos_avg += other_position[1]
                            xvel_avg += velocities[other_boat][0]
                            yvel_avg += velocities[other_boat][1]
                            neighboring_boids += 1

                if neighboring_boids > 0:
                    xpos_avg /= neighboring_boids
                    ypos_avg /= neighboring_boids
                    xvel_avg /= neighboring_boids
                    yvel_avg /= neighboring_boids

                    # Update velocity based on alignment (matching) and cohesion (centering)
                    velocities[boat] = (
                        velocities[boat][0] + (xpos_avg - position[0]) * self.get_parameter('kp_heading').value + (xvel_avg - velocities[boat][0]) * self.get_parameter('kp_velocity').value,
                        velocities[boat][1] + (ypos_avg - position[1]) * self.get_parameter('kp_heading').value + (yvel_avg - velocities[boat][1]) * self.get_parameter('kp_velocity').value
                    )

                # Update velocity based on separation (avoidance)
                velocities[boat] = (
                    velocities[boat][0] + close_dx * self.get_parameter('kp_heading').value,
                    velocities[boat][1] + close_dy * self.get_parameter('kp_heading').value
                )

            # Calculate desired heading based on the updated velocity
            desired_heading = np.arctan2(velocities[boat][1], velocities[boat][0])
            distance_error = np.linalg.norm(np.array([xpos_avg, ypos_avg]) - np.array(position)) - desired_distance
            velocity_ref = self.get_parameter('kp_velocity').value * distance_error

            # Publish the calculated desired heading and velocity reference for the boat
            self.heading_publishers[boat].publish(Float32(data=desired_heading))
            self.velocity_publishers[boat].publish(Float32MultiArray(data=[velocity_ref, 0.0, 0.0]))

            # Log the desired heading and velocity reference for monitoring
            self.get_logger().info(f"{boat} - Desired heading: {desired_heading}, Velocity reference: {velocity_ref}")

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmControllerNode()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
