import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

class VslamDiagnosticNode(Node):
    def __init__(self):
        super().__init__('vslam_diagnostic_node')
        
        # Subscriber to a Bool topic
        self.subscription = self.create_subscription(
            VisualSlamStatus,
            '/visual_slam/status', 
            self.listener_callback,
            10 # QoS
        )
        
        # Publisher for diagnostic status
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # Timer to publish diagnostics every second
        self.timer = self.create_timer(1.0, self.publish_diagnostic_status)

        # Initialize state
        self.last_received_value = False
        self.has_received_message = False

        # Initialize the DiagnosticArray
        self.diagnostics = DiagnosticArray()

        self.max_execution_time = 0.01 # seconds

    def listener_callback(self, msg):
        # Update the received value
        self.last_received_value = msg
        self.has_received_message = True
        self.publish_diagnostic_status()

    def parse_status(self, msg):
        
        if msg.vo_state == 0:
            return "Unknown State", DiagnosticStatus.ERROR
        if msg.vo_state == 1:
            return "Success", DiagnosticStatus.OK
        if msg.vo_state == 2:
            return "Failed", DiagnosticStatus.ERROR
        if msg.vo_state == 3:
            return "Success but invalidated by IMU", DiagnosticStatus.WARN

        return f"unknown error: {msg.vo_state}", DiagnosticStatus.WARN

    def create_blank_diagnostics_msg(self):
        # Initialize the DiagnosticArray
        diagnostics = DiagnosticArray()
        
        # add main status
        vo_state = DiagnosticStatus()
        vo_state.name = "vslam_monitor/vo_state"
        vo_state.message = "No recent messages; assuming False"
        vo_state.level = DiagnosticStatus.WARN
        diagnostics.status.append(vo_state)

        return diagnostics

    def create_diagnostics_msg(self, msg):
        
        # Initialize the DiagnosticArray
        diagnostics = DiagnosticArray()
        
        # add main status
        vo_state = DiagnosticStatus()
        vo_state.name = "vslam_monitor/vo_state"
        vo_state.message, vo_state.level = self.parse_status(msg)
        diagnostics.status.append(vo_state)

        # create exec_time status
        exec_time = DiagnosticStatus()
        exec_time.name = "vslam_monitor/execution_time"
        if msg.node_callback_execution_time < self.max_execution_time:
            exec_time.level = DiagnosticStatus.OK
        else:
            exec_time.level = DiagnosticStatus.WARN

        exec_time.message = f"{msg.node_callback_execution_time} s"
        exec_time.values.append(
                KeyValue(key="node_callback_execution_time",
                    value=f"{msg.node_callback_execution_time} s"))
        exec_time.values.append(
                KeyValue(key="track_execution_time",
                    value=f"{msg.track_execution_time} s"))
        exec_time.values.append(
                KeyValue(key="track_execution_time_mean",
                    value=f"{msg.track_execution_time_mean} s"))
        exec_time.values.append(
                KeyValue(key="track_execution_time_max",
                    value=f"{msg.track_execution_time_max} s"))
        diagnostics.status.append(exec_time)

        # # float64 node_callback_execution_time
        # status = DiagnosticStatus()
        # status.name = "vslam_monitor/node_callback_execution_time"
        # status.message = f"{msg.node_callback_execution_time}"
        # status.level = DiagnosticStatus.OK
        # diagnostics.status.append(status)

        # # float64 track_execution_time
        # status = DiagnosticStatus()
        # status.name = "vslam_monitor/track_execution_time"
        # status.message = f"{msg.track_execution_time}"
        # status.level = DiagnosticStatus.OK if msg.track_execution_time < self.max_execution_time else DiagnosticStatus.WARN
        # diagnostics.status.append(status)

        # # float64 track_execution_time_mean
        # status = DiagnosticStatus()
        # status.name = "vslam_monitor/track_execution_time_mean"
        # status.message = f"{msg.track_execution_time_mean}"
        # status.level = DiagnosticStatus.OK if msg.track_execution_time_mean < self.max_execution_time else DiagnosticStatus.WARN

        # diagnostics.status.append(status)

        # # float64 track_execution_time_max
        # status = DiagnosticStatus()
        # status.name = "vslam_monitor/track_execution_time_max"
        # status.message = f"{msg.track_execution_time_max}"
        # status.level = DiagnosticStatus.OK 
        # diagnostics.status.append(status)

        return diagnostics

    def publish_diagnostic_status(self):
        
        if self.has_received_message:
            self.diagnostics = self.create_diagnostics_msg(self.last_received_value)
        else:
            self.diagnostics = self.create_blank_diagnostics_msg()

        # add a stamp
        self.diagnostics.header.stamp = self.get_clock().now().to_msg()

        # Publish the diagnostic status
        self.diagnostic_publisher.publish(self.diagnostics)

        # Reset the flag
        if self.has_received_message:
            self.has_received_message = False

def main(args=None):
    rclpy.init(args=args)
    node = VslamDiagnosticNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

