import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.declare_parameter('reliability', 'reliable')
        reliability = self.get_parameter('reliability').get_parameter_value().string_value.lower()

        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(Float32, 'temperature_celsius', self.cb, qos)

    def cb(self, msg: Float32):
        self.get_logger().info(f'T={msg.data:.2f} °C')

def main():
    rclpy.init()
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
