import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ActuationPrioPublisher(Node):
    def __init__(self):
        super().__init__('actuation_prio_publisher')  

        # Vessel IDs to Control
        self.vesselids = ['RAS_TN_DB', 'RAS_TN_OR', 'RAS_TN_GR']  # Update if needed
        # No publishers attribute

        # Timer for Control Loop (adjust frequency as needed)
        self.timer = self.create_timer(0.1, self.control_loop)  

    def control_loop(self):
        msg = JointState()  # Create a single message for all vessels

        for vesselid in self.vesselids:
            # Append actuator commands for each vessel to the message
            #SB_aft_thruster_propeller
            msg.name.extend([       'SB_aft_thruster_propeller', 
                                    'PS_aft_thruster_propeller', 
                                    'BOW_thruster_propeller',
                                    'SB_aft_thruster_joint',
                                    'PS_aft_thruster_joint'])
            msg.position.extend([0.0, 0.0, 0.0, 0.0, 0.0])  # Angles (not used)
            msg.velocity.extend([10000.0, 10000.0, 0.0, 0.0, 0.0])  # RPMs
            msg.effort.extend([0.0, 0.0, 0.0, 0.0, 0.0])

        # Publish the message to each vessel topic
        for vesselid in self.vesselids:
            self.create_publisher(JointState, f'/{vesselid}/reference/actuation_prio', 10).publish(msg)

def main(args=None):
    rclpy.init(args=args)
    actuation_prio_publisher = ActuationPrioPublisher()
    rclpy.spin(actuation_prio_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
