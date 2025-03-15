import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/fixed_scan',
            10
        )

    def callback(self, msg):
        print(len(msg.ranges))

def main(args=None):
    rclpy.init(args=args)
    node = ScanFixer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

