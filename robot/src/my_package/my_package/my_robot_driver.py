#!/usr/bin/env python

import rclpy
from example_interfaces.msg import String
from std_msgs.msg import Float32
from my_package.fuzzy_controller import Fuzzy


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__fuzzy_controller = Fuzzy()
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__angular_speed = 0.0
        self.__lineal_speed = 0.0

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(String, 'epuck_controller', self.callback, 1)
        self.__node.create_subscription(Float32, 'distance', self.dist_callback, 1)
        self.__node.get_logger().info("the robot is hearing")

    def dist_callback(self,dist):
        if self.__lineal_speed > 0.0 and dist.data < 1.0:
            self.__node.get_logger().info(str(dist.data*10))
            self.__node.get_logger().info(str(self.__lineal_speed))
            self.__lineal_speed = self.__fuzzy_controller.calculate_speed(dist.data*10,self.__lineal_speed)
            self.__node.get_logger().info(str(round(self.__lineal_speed,2)))

    def callback(self, command):
        MOV_ROB[str(command.data)](self=self)
        
    def move_fw(self):
        self.__lineal_speed = min(self.__lineal_speed + 0.05, 0.10)
    
    def move_bw(self):
        self.__lineal_speed = max(self.__lineal_speed - 0.05, -0.05)

    def move_lf(self):
        self.__angular_speed = min(self.__angular_speed + 0.15, 0.3)
    
    def move_rg(self):
        self.__angular_speed = max(self.__angular_speed- 0.15, -0.3)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        command_motor_left = (self.__lineal_speed - self.__angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (self.__lineal_speed + self.__angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

MOV_ROB = {"a":MyRobotDriver.move_lf,"w":MyRobotDriver.move_fw,"s":MyRobotDriver.move_bw,"d":MyRobotDriver.move_rg}


def main():
    print('Hi from my_package.')


if __name__ == '__main__':
    main()
