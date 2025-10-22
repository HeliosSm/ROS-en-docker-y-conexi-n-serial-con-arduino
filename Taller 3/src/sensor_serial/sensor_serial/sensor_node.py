import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # Parámetros
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('period', 1.0)          # s
        self.declare_parameter('reliability', 'reliable')  # reliable | best_effort

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        period = self.get_parameter('period').get_parameter_value().double_value
        reliability = self.get_parameter('reliability').get_parameter_value().string_value.lower()

        # QoS
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        # Serial
        self.ser = serial.Serial(port, baud, timeout=1)

        # Publisher
        self.pub = self.create_publisher(Int32, 'sensor_data', qos)
        self.timer = self.create_timer(period, self.publish_data)

    def publish_data(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.isdigit():
                msg = Int32()
                msg.data = int(line)
                self.pub.publish(msg)
                self.get_logger().info(f'crudo={msg.data}')
            else:
                if line:
                    self.get_logger().warn(f'dato inválido: "{line}"')
        except Exception as e:
            self.get_logger().error(f'Error serial: {e}')

def main():
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
