import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from influxdb_client import InfluxDBClient, Point

INFLUX_URL = os.getenv('INFLUX_URL', 'http://influxdb:8086')
INFLUX_TOKEN = os.getenv('INFLUX_TOKEN', '')
INFLUX_ORG = os.getenv('INFLUX_ORG', 'wsn')
INFLUX_BUCKET = os.getenv('INFLUX_BUCKET', 'temperatures')

class ExporterNode(Node):
    def __init__(self):
        super().__init__('exporter_node')
        self.declare_parameter('reliability', 'reliable')
        reliability = self.get_parameter('reliability').get_parameter_value().string_value.lower()

        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(Float32, 'temperature_celsius', self.cb, qos)
        if not INFLUX_TOKEN:
            self.get_logger().warn('INFLUX_TOKEN vacío; exportación no funcionará hasta configurarlo.')
        self.client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
        self.write_api = self.client.write_api()

    def cb(self, msg: Float32):
        p = Point("temperature").field("celsius", float(msg.data))
        self.write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=p)
        self.get_logger().info(f'influx={msg.data:.2f} °C')

def main():
    rclpy.init()
    node = ExporterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
