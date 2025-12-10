import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardPublisher:
    def __init__(self, topic_name='keyboard_input'):
        super().__init__('keyboard_publisher')
        
        self.pub = self.create_publisher(String, topic_name, 10)  
        self.running = True
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Keyboard publisher gestart. Gebruik: z,a,e,s. en Ctrl-C = stoppen.')
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    def _loop(self):
        try:
            while rclpy.ok() and self.running:
                key=self._get_key()
                if not key:
                    time.sleep(0.1)
                    continue
                if key == '\x03':  # Ctrl-C
                    break
                msg = String()
                msg.data = key
                self.pub.publish(msg)
                self.get_logger().info(f'Key pressed: {key}')
        finally:
            stop_msg = String()
            stop_msg.data = 'programma gestopt'
            try:
                self.pub.publish(stop_msg)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            Node.destroy_node()
        except Exception as e:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()