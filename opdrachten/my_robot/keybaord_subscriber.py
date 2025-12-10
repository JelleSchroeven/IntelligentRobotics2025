import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

try:
    import serial
except Exception:
    serial = None

class KeyboardSubscriber(Node):
    def __init__(self, topic_name='keyboard_input', serial_port='/dev/ttyACM0', baud=115200):
        super().__init__('keyboard_subscriber')

        self.get_logger().info('Keyboard subscriber gestart, wacht op toetsenbord input...')
        self.sub = self.create_subscription(
            String,
            topic_name,
            self._on_key,
            10)
        
        self.lock = threading.Lock()
        self.last_key = None

        # Rpi serial verbinding met opencr
        self.serial_port = serial_port
        self.baud = baud
        self.ser = None

        #snelheid instellingen
        self.drive_speed = 50
        self.turn_speed = 40
        self.duratie = 0.5  # seconden

        self._init_hardware()

    def _init_hardware(self):
        if serial is None:
            self.get_logger().warning('geen seriële communicatie')
            return
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=1)
            time.sleep(2)  # wacht op seriële verbinding
            self.get_logger().info(f'Seriële verbonde op {self.serial_port} met baud {self.baud}')

            self._send_serial('V 0 0') #motren op0 bij start
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Fout bij seriële verbinding: {e}')
    
    def _send_serial(self, command: str):
    
        if self.ser:
            try:
                self.ser.write((command.strip() + '\n').encode())
                self.get_logger().info(f'Sent to serial: {command}')
            except Exception as e:
                self.get_logger().error(f'Fout bij verzenden naar serial poort: {e}')
        else:
            self.get_logger().warning('Seriële poort niet beschikbaar')

    def _on_key(self, msg: String):
        key = msg.data
        if not isinstance(key, str) or len(key) == 0:
            return
        key = key.strip()
        self.get_logger().info(f'Ontvangen key: {key}')
        with self.lock:
            self.last_key = key 
        
        #movement keys
        if key == 'z':
            self._move_forward()
        elif key == 's':
            self._move_backward()
        elif key == 'a':
            self._turn_left()
        elif key == 'e':
            self._turn_right()
        elif key == 'STOP':
            self._stop_motors()
        else: 
            # key niet herkent
            self.get_logger().info(f'Onbekende key: {key}')
    
    def _move_forward(self):
        left_motor_speed = int(self.drive_speed)
        right_motor_speed = int(self.drive_speed)
        cmd = f'D {left_motor_speed} {right_motor_speed} {self.duratie}'
        self._send_serial(cmd)
        self.get_logger().info('Robot beweegt vooruit voor {self.duratie} seconden')
    
    def _move_backward(self):
        left_motor_speed = int(self.drive_speed)
        right_motor_speed = int(self.drive_speed)
        cmd = f'D {left_motor_speed} {right_motor_speed} {self.duratie}'
        self._send_serial(cmd)
        self.get_logger().info('Robot beweegt achteruit voor {self.duratie} seconden')

    def _turn_left(self):
        left_motor_speed = int(self.turn_speed)
        right_motor_speed = int(self.turn_speed)
        cmd = f'D {left_motor_speed} {right_motor_speed} {self.duratie}'
        self._send_serial(cmd)
        self.get_logger().info('Robot draait naar links')

    def _turn_right(self):
        self.get_logger().info('Robot draait naar rechts')
        left_motor_speed = int(self.turn_speed)
        right_motor_speed = int(self.turn_speed)
        cmd = f'D {left_motor_speed} {right_motor_speed} {self.duratie}'
        self._send_serial(cmd)

    def _stop_motors(self):
        cmd = f'V 0 0'
        self._send_serial(cmd)
        self.get_logger().info('Robot stopt')


    def destroy_node(self):
        try:
            self.get_logger().info('Keyboard subscriber wordt afgesloten.')
            self.send_serial('V 0 0')  # stop motors
            if self.ser:
                try:
                    self.ser.close()
                except Exception as e:
                    self.get_logger().error(f'Fout bij sluiten seriële poort: {e}')
        except Exception as e:
            pass
        super().destroy_node()
        


def main(args=None):
    rclpy.init(args=args)
    keyboard_subscriber = KeyboardSubscriber()
    try:
        rclpy.spin(keyboard_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            Node.destroy_node()
        except Exception as e:
            pass
        rclpy.shutdown()