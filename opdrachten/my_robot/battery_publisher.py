import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        try: 
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Serial connection established on /dev/tty<ACM0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            raise
    
        self.publisher = self.create_publisher(Float32, 'battery_voltage', 10)   
        self.timer = self.create_timer(60, self.read_battery_voltage)
        self.read_battery_voltage() #geeft voltage bij opstart
    
    def read_battery_voltage(self):
        msg = Float32()
        try:
            self.ser.write(b'B\n')
        
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                self.get_logger().warn('Geen voltage ontvangen van serial')
                msg.data = -1.0
            elif line.startswith("BatteryVoltage:"):
                try:
                    raw = line.split(":")[1].split(" ")[0]
                    voltage = float(raw)
                    msg.data = voltage
                    self.get_logger().info(f'Published battery voltage: {voltage:.2f}V')
                except Exception as e:
                    self.get_logger().warn(f'Kon voltage niet parsen uit "{line}": {e}')
                    msg.data = -1.0
            else:
                self.get_logger().warn(f'Ongeldige respons: "{line}"')
                msg.data = -1.0
        except Exception as e:
            self.get_logger().error(f'Error reading battery voltage: {e}')
            msg.data = -1.0

        self.publisher.publish(msg)



def main(args=None):
        rclpy.init(args=args)
        battery_publisher = BatteryPublisher()
        rclpy.spin(battery_publisher)
        battery_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
        