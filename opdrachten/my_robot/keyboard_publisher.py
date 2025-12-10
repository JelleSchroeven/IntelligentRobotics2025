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

    