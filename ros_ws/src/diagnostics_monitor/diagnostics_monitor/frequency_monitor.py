
import rclpy 
from rclpy.node import Node
from diagnostic_updater import Updater, HeaderlessTopicDiagnostic, FrequencyStatusParam, TimeStampStatusParam
from rclpy import qos

# topic type
from std_msgs.msg import String

# topic type
# from rclpy.anymsg import AnyMsg
from rosidl_runtime_py.utilities import get_message

class FrequencyMonitor(Node):
    def __init__(self):

        super().__init__('frequency_monitor')

        # declare params
        self.declare_parameter('topic_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('topic_type', rclpy.Parameter.Type.STRING)
        self.declare_parameter('min_frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('time_window', 0.1)
        self.declare_parameter('tolerance', 0.0)

        # get params
        self.topic_name = self.get_parameter('topic_name').value
        self.topic_type = self.get_parameter('topic_type').value
        self.min_frequency = self.get_parameter('min_frequency').value
        self.max_frequency = self.get_parameter('max_frequency').value
        self.window_size = self.get_parameter('window_size').value
        self.time_window = self.get_parameter('time_window').value
        self.tolerance = self.get_parameter('tolerance').value

        # create an updater
        self.updater = Updater(self)
        self.updater.setHardwareID(f"frequency_monitor/{self.topic_name}")

        self.freq_bounds = {
                'min': self.min_frequency,
                'max': self.max_frequency
                }

        frequency_status_param = FrequencyStatusParam(
                self.freq_bounds, 
                self.tolerance,
                self.window_size)
        
        # add a topic diagnostic
        self.topic_diagnostic = HeaderlessTopicDiagnostic(
                self.topic_name, 
                self.updater, 
                frequency_status_param
                )

        # create a subscriber for the topic
        self.subscription = self.create_subscription(
                get_message(self.topic_type),
                self.topic_name,
                self.message_callback,
                qos.qos_profile_sensor_data,
                raw=True
                )

    def message_callback(self, msg):
        self.topic_diagnostic.tick()

    def update_diagnostics(self):
        self.updater.update()


def main(args=None):
    rclpy.init(args=args)

    node = FrequencyMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("keyboard interrupt")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

