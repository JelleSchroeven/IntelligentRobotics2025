import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'battery_voltage',
            self.battery_callback,
            10)
        self.subscription

    def battery_callback(self, msg):
        voltage = msg.data
        if voltage == -1.0:
            self.get_logger().warn('No battery voltage received.')
        else:
            self.get_logger().info(f'Battery voltage: {voltage:.2f}V')
            if voltage < 11.5:
                self.get_logger().warn('Battery voltage low recharge ASAP.')

def main(args=None):
        rclpy.init(args=args)
        battery_subscriber = BatterySubscriber()
        rclpy.spin(battery_subscriber)
        battery_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()