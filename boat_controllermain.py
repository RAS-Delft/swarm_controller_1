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
                #('lightblue_boat', 'RAS_TN_LB'),
                #('red_boat', 'RAS_TN_RE'),
                #('yellow_boat', 'RAS_TN_YE'),
                #('purple_boat', 'RAS_TN_PU'),
                ('desired_distance', 2.0),
                ('separation_distance', 1.5),
                ('kp_heading', 0.005),
                ('kp_velocity', 0.05),
                ('matching_factor', 0.2),
                ('avoid_factor', 0.5),
                ('centering_factor', 0.005),
                ('min_speed', 0.01),
                ('max_speed', 0.5)  # Reduced max speed for better control
            ]
        )

        # Boat identifiers
        self.boats = [
            self.get_parameter('orange_boat').value,
            self.get_parameter('dark_blue_boat').value,
            self.get_parameter('green_boat').value,
            #self.get_parameter('lightblue_boat').value,
            #self.get_parameter('yellow_boat').value
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
        matching_factor = self.get_parameter('matching_factor').value
        avoid_factor = self.get_parameter('avoid_factor').value
        centering_factor = self.get_parameter('centering_factor').value
        min_speed = self.get_parameter('min_speed').value
        max_speed = self.get_parameter('max_speed').value

        positions = {boat: (pose.translation.x, pose.translation.y) for boat, pose in self.poses.items()}
        velocities = {boat: np.array([vel[0], vel[1]]) for boat, vel in self.velocities.items()}

        for boat in self.boats:
            position = positions[boat]
            velocity = velocities[boat]

            # Initialize forces
            separation_force = np.array([0.0, 0.0])
            alignment_force = np.array([0.0, 0.0])
            cohesion_force = np.array([0.0, 0.0])
            neighbor_count = 0
            close_neighbor_count = 0

            for other_boat in self.boats:
                if other_boat != boat:
                    other_position = positions[other_boat]
                    other_velocity = velocities[other_boat]
                    distance = np.linalg.norm(np.array(position) - np.array(other_position))

                    # Alignment
                    alignment_force += other_velocity
                    neighbor_count += 1

                    # Cohesion
                    cohesion_force += np.array(other_position)

                    # Separation
                    if distance < separation_distance:
                        separation_force += (np.array(position) - np.array(other_position)) / (distance**2)
                        close_neighbor_count += 1

            if neighbor_count > 0:
                alignment_force /= neighbor_count
                alignment_force = (alignment_force - velocity) * matching_factor

                cohesion_force /= neighbor_count
                cohesion_force = (cohesion_force - np.array(position)) * centering_factor

            if close_neighbor_count > 0:
                separation_force = separation_force * avoid_factor
            else:
                separation_force = np.array([0.0, 0.0])  # No separation force if no neighbors are too close

            # Calculate the desired velocity
            desired_velocity = velocity + alignment_force + cohesion_force + separation_force
            desired_speed = np.linalg.norm(desired_velocity)
            if desired_speed > 0:
                desired_heading = np.arctan2(desired_velocity[1], desired_velocity[0])
            else:
                desired_heading = self.headings[boat]

            # Constrain the speed
            if desired_speed > max_speed:
                desired_velocity = (desired_velocity / desired_speed) * max_speed
                desired_speed = max_speed
            elif desired_speed < min_speed:
                desired_velocity = (desired_velocity / desired_speed) * min_speed
                desired_speed = min_speed

            # Publish references
            self.heading_publishers[boat].publish(Float32(data=desired_heading))
            self.velocity_publishers[boat].publish(Float32MultiArray(data=[desired_speed, 0.0, 0.0]))

            # Logging
            self.get_logger().info(f"{boat} - Desired heading: {desired_heading}, Velocity reference: {desired_speed}")

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmControllerNode()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
