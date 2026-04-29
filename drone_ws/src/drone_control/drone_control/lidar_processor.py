import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LidarProcessor(Node):
    NUM_SECTORS = 8

    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/sector_distances', 10)
        self.get_logger().info('LiDAR processor started, waiting for /scan ...')

    def cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        n = len(ranges)
        sz = max(1, n // self.NUM_SECTORS)
        mins = []
        for i in range(self.NUM_SECTORS):
            sector = ranges[i*sz:(i+1)*sz]
            valid = [r for r in sector if math.isfinite(r) and r > 0.05]
            mins.append(min(valid) if valid else float('inf'))
        self.pub.publish(Float32MultiArray(data=mins))
        self.get_logger().info(
            f'Sectors(m): F={mins[0]:.2f} FR={mins[1]:.2f} R={mins[2]:.2f} '
            f'BR={mins[3]:.2f} B={mins[4]:.2f} BL={mins[5]:.2f} '
            f'L={mins[6]:.2f} FL={mins[7]:.2f}')

def main():
    rclpy.init()
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
