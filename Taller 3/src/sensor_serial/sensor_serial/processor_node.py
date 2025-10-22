import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        self.declare_parameter('reliability', 'reliable')

        reliability = self.get_parameter('reliability').get_parameter_value().string_value.lower()
        qos_sub = QoSProfile(depth=10)
        qos_sub.history = HistoryPolicy.KEEP_LAST
        qos_sub.reliability = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        qos_pub = QoSProfile(depth=10)
        qos_pub.history = HistoryPolicy.KEEP_LAST
        qos_pub.reliability = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(Int32, 'sensor_data', self.cb, qos_sub)
        self.pub = self.create_publisher(Float32, 'temperature_celsius', qos_pub)

    def cb(self, msg: Int32):
        # Escala ejemplo 0..1023 => 0..100 °C (ajusta si tu sensor real difiere)
        temp = (msg.data / 1023.0) * 100.0
        out = Float32()
        out.data = float(f'{temp:.2f}')
        self.pub.publish(out)
        self.get_logger().info(f'temp={out.data:.2f} °C')

def main():
    rclpy.init()
    node = ProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
