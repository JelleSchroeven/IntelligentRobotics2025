import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time


class KeyboardSubscriber(Node):
    def __init__(self, topic_name='keyboard_input'):
        super().__init__('keyboard_subscriber')

        self.get_logger().info('Keyboard subscriber gestart, wacht op toetsenbord input...')
        self.sub = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        
        self._init_hardware()

    def _init_hardware(self):
        if GPIO is not None:
            self.get_logger().info('GPIO library gevonden — start pins .')
        else:
            self.get_logger().info('Geen GPIO beschikbaar — motor callbacks zijn placeholders.')

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
            self.get_logger().info(f'Onbekende toets: {key}')
    
    def _move_forward(self):
        self.get_logger().info('Robot beweegt vooruit')
    
    def _move_backward(self):
        self.get_logger().info('Robot beweegt achteruit')

    def _turn_left(self):
        self.get_logger().info('Robot draait naar links')

    def _turn_right(self):
        self.get_logger().info('Robot draait naar rechts')

    def _stop_motors(self):
        self.get_logger().info('Robot stopt')


    def destroy_node(self):
        self.get_logger().info('Keyboard subscriber wordt afgesloten.')
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