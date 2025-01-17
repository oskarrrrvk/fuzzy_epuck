#!/usr/bin python3


import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Publisher_Node(Node):

    def __init__(self):
        super().__init__("epuck_controller")
        self.movements = ["w","a","s","d", "x"]
        self.publisher = self.create_publisher(String,'epuck_controller',10)
        self.timer = self.create_timer(1,self.publish_movement)
        self.get_logger().info("controls: a,w,s,d. Press Enter to send the command. Press x to exit")

    def publish_movement(self):
        msg = String()
        msg.data = self.read_keyboard()
        if msg.data == "x":
            self.shutdown()
        elif msg.data != " ":
            self.publisher.publish(msg)
    
    def read_keyboard(self):
        command = input()
        if command in self.movements:
            return command
        return " "

def main(args=None):
    try:
        rclpy.init(args=args)
        public = Publisher_Node()
        rclpy.spin(public)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
