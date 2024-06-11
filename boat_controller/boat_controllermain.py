import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from tf2_msgs.msg import TFMessage

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

def ros2_transform_to_yaw(transform:TransformStamped):
    """uses the quaternion from a ros2 transform to calculate the yaw"""
    x = transform.transform.rotation.x
    y = transform.transform.rotation.y
    z = transform.transform.rotation.z
    w = transform.transform.rotation.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)

    a = np.arctan2(t3, t4)
    # map between 0 and 2pi
    if a < 0:
        a += 2*np.pi

    return a


class SwarmControllerNode(Node):

    def __init__(self):
        super().__init__('swarm_controller')
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Get the necessary parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('orange_boat', 'RAS_TN_OR'),
                ('dark_blue_boat', 'RAS_TN_DB'),
                ('green_boat', 'RAS_TN_GR'),
                ('lightblue_boat', 'RAS_TN_LB'),
                ('red_boat', 'RAS_TN_RE'),
                ('yellow_boat', 'RAS_TN_YE'),
                ('purple_boat', 'RAS_TN_PU'),
                ('desired_distance', 4),
                ('separation_distance', 3),
                ('matching_factor', 0.5),
                ('avoid_factor', 0.3),
                ('centering_factor', 0.005),
                ('goal_factor', 0.025),  # New parameter for goal seeking
                ('min_speed', 0.01),
                ('max_speed', 0.2),  # Reduced max speed for better control
                ('max_heading_rate', 0.6)  # Maximum rate of change for heading (radians per update)
            ]
        )

        # Boat identifiers
        self.boats = [
            self.get_parameter('orange_boat').value,
            self.get_parameter('dark_blue_boat').value,
            self.get_parameter('green_boat').value,
            self.get_parameter('lightblue_boat').value,
            self.get_parameter('yellow_boat').value,
            self.get_parameter('purple_boat').value,
            self.get_parameter('red_boat').value
        ]

        # Set up subscribers
        self.pose_subscribers = {
            boat: self.create_subscription(
                TransformStamped,
                f"/{boat}/pose",
                lambda msg, boat=boat: self.pose_callback(msg, boat),
                custom_qos_profile
            ) for boat in self.boats
        }
        self.heading_subscribers = {
            boat: self.create_subscription(
                Float32,
                f"/{boat}/telemetry/heading",
                lambda msg, boat=boat: self.heading_callback(msg, boat),
                custom_qos_profile
            ) for boat in self.boats
        }
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            custom_qos_profile
        )

        self.velocity_subscribers = {
            boat: self.create_subscription(
                Float32MultiArray,
                f"/{boat}/state/velocity",
                lambda msg, boat=boat: self.velocity_callback(msg, boat),
                custom_qos_profile
            ) for boat in self.boats
        }

        # Subscriber for the clicked point in rviz2
        self.goal_subscriber = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            custom_qos_profile
        )

        # Set up publishers
        self.heading_publishers = {
            boat: self.create_publisher(
                Float32,
                f"/{boat}/reference/heading",
                custom_qos_profile
            ) for boat in self.boats
        }
        self.velocity_publishers = {
            boat: self.create_publisher(
                Float32MultiArray,
                f"/{boat}/reference/velocity",
                custom_qos_profile
            ) for boat in self.boats
        }

        # # Create distance publishers
        # self.distance_publishers = {
        #     (boat1, boat2): self.create_publisher(
        #         Float32,
        #         f"/distance/{boat1}_{boat2}",
        #         custom_qos_profile
        #     ) for i, boat1 in enumerate(self.boats) for boat2 in self.boats[i+1:]
        # }

        # Initialize state variables
        self.poses = {boat: None for boat in self.boats}
        self.headings = {boat: None for boat in self.boats}
        self.velocities = {boat: None for boat in self.boats}
        self.goal_position = None  # Variable to store the clicked goal position

    """
    Registers ships' poses in the poses dictionary and checks if all data is received.
    """
    def tf_callback(self, msg):
        for transform in msg.transforms:
            for i in range(0, self.max_boat_index):
                if self.base_link_name[i] == transform.child_frame_id:
                    self.poses[i] = transform
                    self.headings[i] = ros2_transform_to_yaw(transform)
                    if i == self.boat_nr:
                        self.calculate_and_publish_references()
        
    def velocity_callback(self, msg, boat):
        self.velocities[boat] = msg.data

    def goal_callback(self, msg):
        self.goal_position = np.array([msg.point.x, msg.point.y])
        self.get_logger().info(f"New goal position: {self.goal_position}")

    def calculate_and_publish_references(self):

        desired_distance = self.get_parameter('desired_distance').value
        separation_distance = self.get_parameter('separation_distance').value
        matching_factor = self.get_parameter('matching_factor').value
        avoid_factor = self.get_parameter('avoid_factor').value
        centering_factor = self.get_parameter('centering_factor').value
        goal_factor = self.get_parameter('goal_factor').value  # Get the goal seeking factor
        min_speed = self.get_parameter('min_speed').value
        max_speed = self.get_parameter('max_speed').value
        max_heading_rate = self.get_parameter('max_heading_rate').value

        positions = {boat: (pose.translation.x, pose.translation.y) for boat, pose in self.poses.items() if pose is not None}
        velocities = {boat: np.array([vel[0], vel[1]]) for boat, vel in self.velocities.items() if vel is not None}

        for boat in self.boats:
            if boat in positions:
                position = positions[boat]
            else:
                position = None
            
            if position is not None:
                if boat in velocities:
                    velocity = velocities[boat]
                else:
                    velocity = np.array([0.0, 0.0])

                # Initialize forces
                separation_force = np.array([0.0, 0.0])
                alignment_force = np.array([0.0, 0.0])
                cohesion_force = np.array([0.0, 0.0])
                goal_force = np.array([0.0, 0.0])
                neighbor_count = 0
                close_neighbor_count = 0

                for other_boat in self.boats:
                    if other_boat != boat:
                        if other_boat in positions:
                            other_position = positions[other_boat]
                            distance = np.linalg.norm(np.array(position) - np.array(other_position))

                            # Apply different logic based on distance
                            if distance < separation_distance:
                                separation_force += (np.array(position) - np.array(other_position)) * ((separation_distance - distance)**2)
                                close_neighbor_count += 1
                            elif other_boat in velocities:
                                other_velocity = velocities[other_boat]
                                if separation_distance <= distance <= desired_distance:
                                    alignment_force += other_velocity
                                    neighbor_count += 1
                                else:
                                    alignment_force += other_velocity
                                    cohesion_force += (np.array(other_position) - np.array(position)) * ((distance - desired_distance) / desired_distance)
                                    neighbor_count += 1
                            
                            if separation_distance <= distance <= desired_distance:
                                cohesion_force = np.array([0.0, 0.0])

                if neighbor_count > 0:
                    alignment_force /= neighbor_count
                    alignment_force = (alignment_force - velocity) * matching_factor

                    cohesion_force /= neighbor_count
                    cohesion_force = cohesion_force * centering_factor

                if close_neighbor_count > 0:
                    separation_force = separation_force * avoid_factor
                else:
                    separation_force = np.array([0.0, 0.0])

                if self.goal_position is not None:
                    goal_force = (self.goal_position - np.array(position)) * goal_factor
            
                desired_velocity = velocity + alignment_force + cohesion_force + separation_force + goal_force
                desired_speed = np.linalg.norm(desired_velocity)

                if desired_speed > 0:
                    desired_heading = np.arctan2(desired_velocity[1], desired_velocity[0])
                else:
                    desired_heading = self.headings[boat]

                if boat in self.headings and self.headings[boat] is not None:
                    current_heading = self.headings[boat]
                    heading_change = desired_heading - current_heading
                    if heading_change > np.pi:
                        heading_change -= 2 * np.pi
                    elif heading_change < -np.pi:
                        heading_change += 2 * np.pi

                    if abs(heading_change) > max_heading_rate:
                        heading_change = np.sign(heading_change) * max_heading_rate

                    desired_heading = current_heading + heading_change

                if desired_speed > max_speed:
                    desired_velocity = (desired_velocity / desired_speed) * max_speed
                    desired_speed = max_speed
                elif desired_speed < min_speed and desired_speed != 0:
                    desired_velocity = (desired_velocity / desired_speed) * min_speed
                    desired_speed = min_speed

                if close_neighbor_count > 0:
                    separation_velocity = separation_force + velocity
                    separation_speed = np.linalg.norm(separation_velocity)
                    if separation_speed > max_speed:
                        separation_velocity = (separation_velocity / separation_speed) * max_speed
                    elif separation_speed < min_speed:
                        separation_velocity = (separation_velocity / separation_speed) * min_speed
                    desired_velocity = separation_velocity

                msg_heading = Float32(data=desired_heading)
                msg_speed = Float32MultiArray(data=[desired_speed, 0.0, 0.0])

                if desired_heading is not None:
                    self.heading_publishers[boat].publish(msg_heading)

                if desired_speed is not None:
                    self.velocity_publishers[boat].publish(msg_speed)

                self.get_logger().info(f"{boat} - Desired heading: {desired_heading}, Velocity reference: {desired_speed}")

    #     if all(boat in positions for boat in self.boats):
    #         self.publish_distances(positions)

    # def publish_distances(self, positions):
    #     for (boat1, boat2), publisher in self.distance_publishers.items():
    #         distance = np.linalg.norm(np.array(positions[boat1]) - np.array(positions[boat2]))
    #         publisher.publish(Float32(data=distance))


def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmControllerNode()
    rclpy.spin(swarm_controller)
    swarm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
