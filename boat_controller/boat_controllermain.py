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
                ('max_heading_rate', 0.6),  # Maximum rate of change for heading (radians per update)
                ('base_link_suffix', '/base_link')
            ]
        )

        # Boat identifiers
        self.boatNames = [
            self.get_parameter('orange_boat').value,
            self.get_parameter('dark_blue_boat').value,
            self.get_parameter('green_boat').value,
            self.get_parameter('lightblue_boat').value,
            self.get_parameter('yellow_boat').value,
            self.get_parameter('purple_boat').value,
            self.get_parameter('red_boat').value
        ]

        # Set max_boat_index to the number of boats
        self.n_boats = len(self.boatNames)

        # Figure out my name as namespace (remove the leading slash)
        self.vessel_id = self.get_namespace().replace("/", "")

        # Figure out which of the self.boatNames I am, and set it to self.boat_nr
        self.boat_nr = self.boatNames.index(self.vessel_id)

        # Print to inform which boat this is, and if detection went well
        self.get_logger().info(f"Vessel ID: {self.vessel_id}, Boat number: {self.boat_nr}")

        # set array of names of the frames that are the hulls of the boats
        self.base_link_name = [f"/{boat}{self.get_parameter('base_link_suffix').value}" for boat in self.boatNames]

        # Make 1 pose subscriber on /tf
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            custom_qos_profile
        )

        self.velocity_subscribers = []
        # Create subscribers and store them in the list
        for boatnr in range(self.n_boats):
            self.velocity_subscribers.append(
                self.create_subscription(
                    Float32MultiArray,
                    f"/{self.boatNames[boatnr]}/state/velocity",
                    lambda msg, boat=boatnr: self.velocity_callback(msg, boat),
                    custom_qos_profile ) )

        # Subscriber for the clicked point in rviz2
        self.goal_subscriber = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10
        )

        # Set up output publishers
        self.heading_ref_publisher = self.create_publisher(
                Float32,
                f"/{self.vessel_id}/reference/heading",
                custom_qos_profile)

        self.velocity_ref_publisher = self.create_publisher(
                Float32MultiArray,
                f"/{self.vessel_id}/reference/velocity",
                custom_qos_profile) 

        # Initialize state variables, which are updated by the callbacks.
        # They are 'None' until they are received and replaced with a value.
        self.poses = [None for _ in range(self.n_boats)] # Datatype: array of TransformStamped
        self.headings = [None for _ in range(self.n_boats)] # Datatype: array of float
        self.velocities = [None for _ in range(self.n_boats)] # Datatype: array of Float32MultiArray
        self.goal_position = None  # Variable to store the clicked goal position, Datatype: 1x2 numpy array

        # make a list of counters for the number of callbacks ran
        self.tr_tf_callback = 0
        self.tr_tf_callback_ship_detected = 0
        self.tr_velocity_callback_total = 0
        self.tr_calculate_and_publish_references = 0

        # make a timer that periodically reports status at 0.2 Hz
        self.create_timer(5.0, self.report_status)
        self.last_timestamp_reported = self.get_clock().now()

    def report_status(self):
        now = self.get_clock().now()
        dt = now - self.last_timestamp_reported

        f_tf = float(self.tr_tf_callback) / dt.nanoseconds * 1e-9
        f_tf_ships = float(self.tr_tf_callback_ship_detected) / dt.nanoseconds * 1e-9
        f_velocity = float(self.tr_velocity_callback_total) / dt.nanoseconds * 1e-9
        f_runcontrols = float(self.tr_calculate_and_publish_references) / dt.nanoseconds * 1e-9

        # report the status of the calculated frequencies
        self.get_logger().info(f"Diagnostics: f_tf: {f_tf:.2f} Hz, f_tf_ships: {f_tf_ships:.2f} Hz, f_velocity: {f_velocity:.2f} Hz, f_runcontrols: {f_runcontrols:.2f} Hz")

        # reset the counters
        self.tr_tf_callback = 0
        self.tr_tf_callback_ship_detected = 0
        self.tr_velocity_callback_total = 0
        self.tr_calculate_and_publish_references = 0
        self.last_timestamp_reported = now




        

    """
    Registers ships' poses in the poses dictionary and checks if all data is received.
    """
    def tf_callback(self, msg: TFMessage):
        self.tr_tf_callback += 1
        for transform in msg.transforms:
            for i in range(0, self.n_boats):
                if self.base_link_name[i] == transform.child_frame_id:
                    self.tr_tf_callback_ship_detected += 1
                    self.poses[i] = transform
                    self.headings[i] = ros2_transform_to_yaw(transform)
                    if i == self.boat_nr: # Only control if its own pose is received
                        self.calculate_and_publish_references()
        
    def velocity_callback(self, msg: Float32MultiArray, boatnr: int):
        self.tr_velocity_callback_total += 1
        self.velocities[boatnr] = msg.data

    def goal_callback(self, msg:PointStamped):
        self.goal_position = np.array([msg.point.x, msg.point.y])
        self.get_logger().info(f"New goal position: {self.goal_position}")

    def calculate_and_publish_references(self):
        self.tr_calculate_and_publish_references += 1
        desired_distance = self.get_parameter('desired_distance').value
        separation_distance = self.get_parameter('separation_distance').value
        matching_factor = self.get_parameter('matching_factor').value
        avoid_factor = self.get_parameter('avoid_factor').value
        centering_factor = self.get_parameter('centering_factor').value
        goal_factor = self.get_parameter('goal_factor').value  # Get the goal seeking factor
        min_speed = self.get_parameter('min_speed').value
        max_speed = self.get_parameter('max_speed').value
        max_heading_rate = self.get_parameter('max_heading_rate').value

        # Initialize forces
        separation_force = np.array([0.0, 0.0])
        alignment_force = np.array([0.0, 0.0])
        cohesion_force = np.array([0.0, 0.0])
        goal_force = np.array([0.0, 0.0])
        neighbor_count = 0
        close_neighbor_count = 0

        # Interrupt if there is not a pose known for this ship
        if self.poses[self.boat_nr] is None:
            return
        
        # Loop through all boats
        for boatnr in range(self.n_boats):
            # only proceed if its not self
            if self.boatNames[boatnr] != self.vessel_id:
                # Check if the pose is not equal to None
                if self.poses[boatnr] is not None:

                    # Calculate distance between boats
                        

                    position = np.array([self.poses[self.boat_nr].transform.translation.x, self.poses[self.boat_nr].transform.translation.y])
                    other_position = np.array([self.poses[boatnr].transform.translation.x, self.poses[boatnr].transform.translation.y])

                    distance = np.linalg.norm(position - other_position)

                    # Apply different logic based on distance
                    if distance < separation_distance:
                        separation_force += (np.array(position) - np.array(other_position)) * ((separation_distance - distance)**2)
                        close_neighbor_count += 1
                    # elseif other boat has nonzero velocity
                    elif self.velocities[boatnr] is not None:
                        other_velocity = self.velocities[boatnr]
                        if separation_distance <= distance <= desired_distance:
                            alignment_force += other_velocity
                            neighbor_count += 1
                        else:
                            alignment_force += other_velocity
                            if not separation_distance <= distance <= desired_distance:
                                cohesion_force += (other_position - position) * ((distance - desired_distance) / desired_distance)
                            neighbor_count += 1

        if neighbor_count > 0:
            alignment_force /= neighbor_count
            if self.velocities is not None:
                alignment_force = (alignment_force - self.velocities[self.boat_nr]) * matching_factor

            cohesion_force /= neighbor_count
            cohesion_force = cohesion_force * centering_factor

        if close_neighbor_count > 0:
            separation_force = separation_force * avoid_factor
        else:
            separation_force = np.array([0.0, 0.0])

        if self.goal_position is not None:
            goal_force = (self.goal_position - np.array(position)) * goal_factor
    
        if self.velocities[self.boat_nr] is not None:
            velocity = np.array(self.velocities[self.boat_nr])
        else:
            velocity = np.array([0.0, 0.0])
        desired_velocity = velocity + alignment_force + cohesion_force + separation_force + goal_force
        desired_speed = np.linalg.norm(desired_velocity)

        if desired_speed > 0:
            desired_heading = np.arctan2(desired_velocity[1], desired_velocity[0])
        else:
            desired_heading = self.headings[self.boat_nr]

        ## check if the boat has a not none heading
        if self.headings[self.boat_nr] is not None:
            current_heading = self.headings[self.boat_nr]
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
            self.heading_ref_publisher.publish(msg_heading)

        if desired_speed is not None:
            self.velocity_ref_publisher.publish(msg_speed)

        self.get_logger().info(f"{self.base_link_name} Published:- Desired heading: {desired_heading}, Velocity reference: {desired_speed}")

def main(args=None):
    rclpy.init(args=args)
    swarm_controller = SwarmControllerNode()

    try:
        rclpy.spin(swarm_controller)
    except KeyboardInterrupt:
        print("Node interrupted by user.")

if __name__ == '__main__':
    main()
