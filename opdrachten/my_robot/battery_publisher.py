import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class BatteryPublisher(Node):
    def __init__(self):

        try: 
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)
            self.get_logger().info('Serial connection established on /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            raise
    
        self.publisher = self.create_publisher(Float32, 'battery_percentage', 10)   
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def read_battery_voltage(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()

            if not line:
                return
            voltage = float(line)
            msg = Float32()
            msg.data = voltage
            self.publisher.publish(msg)
            self.get_logger().info(f'Published battery voltage: {voltage:.2f}V')
        except ValueError as e:
            #negeer verkeerde waarden
            return
        except Exception as e:
            self.get_logger().error(f'Error reading battery voltage: {e}')
    
    def main(args=None):
        rclpy.init(args=args)
        battery_publisher = BatteryPublisher()
        rclpy.spin(battery_publisher)
        battery_publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
        