#!/usr/bin/env python


from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from rclpy.node import Node
import rclpy

MAX_RANGE = 0.15

class Distance(Node):
    def __init__(self):
        super().__init__("distance")

        self.__left_sensor = 0.0
        self.__right_sensor = 0.0

        self.__publisher = self.create_publisher(Float32, 'distance', 2)
        self.timer = self.create_timer(0.5,self.__send_distance)

        self.create_subscription(Range, '/e_puck/ps0', self.__left_sensor_callback, 1)
        self.create_subscription(Range, '/e_puck/ps7', self.__right_sensor_callback, 1)

        self.get_logger().info("distance sensors are activated")

    def __left_sensor_callback(self,dist):
        self.__left_sensor = dist.range

    def __right_sensor_callback(self,dist):
        self.__right_sensor = dist.range

    def __send_distance(self):
        res = Float32()
        res.data = min(self.__left_sensor, self.__right_sensor)
        self.__publisher.publish(res)


def main(args=None):
    rclpy.init(args=args)
    dist = Distance()
    rclpy.spin(dist)
    dist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
