#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import time

from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Publisher(Node):
    def __init__(self):
        super().__init__("publisher")
        self.i=0
        self.yaw = 0.0
        self.inital_pose = [-2.0,2.0]
        self.tempo_atual= ((self.get_clock().now()).nanoseconds /1e9)
        self.tempo_anterior = self.tempo_atual
        self.robotPos = self.inital_pose
        self.current_pos = self.inital_pose
        self.goals=[[(2.2,2.2), -1], [(2.15,-2.15), 1], [(-2.16,-2.16), 1], [(-2.0,1.2), 1]]
        self.zone_F = [0.0]
        self.zone_L = [0.0]
        self.zone_B = [0.0]
        self.zone_R = [0.0]
        self.points = []
        self.estado = 0
        self.j = 1
        self.side_back = [1, -1, 1, -1]

        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laserScan_subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("Published Walk")

    def scan_callback(self, data: LaserScan):
        self.maxRange = data.range_max
        self.minRange = data.range_min
        self.ranges = data.ranges
        zone = np.array(data.ranges)
        self.zone_F = np.concatenate((zone[0:45], zone[315:360])) 
        self.zone_L = zone[46:134]  
        self.zone_B = zone[135:225]
        self.zone_R = zone[226:314]
        if self.i < len(self.goals):
            if self.estado == 0:
                self.tempo_atual= ((self.get_clock().now()).nanoseconds /1e9)
                self.go_to(self.goals[self.i])
            else:
                if (abs(self.robotPos[0] - (-2.0)) > 0.3) or (abs(self.robotPos[1] - (2.0)) > 0.3):
                    self.go_to([self.points[len(self.points)-self.j],self.side_back[self.i]])
                else:
                    self.estado = 0
                    self.i = self.i+1
                    self.j = 1
                    self.points = [(-2.0, 2.0)]
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = 0.0
                    cmd_vel_msg.angular.z = 0.0
                    self.publisher.publish(cmd_vel_msg)
                    time.sleep(2)


    def odom_callback(self, msg: Odometry):
        robotPos_x = msg.pose.pose.position.x
        robotPos_y = msg.pose.pose.position.y
        self.robotPos = robotPos_x, robotPos_y
        (_, _, self.yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        
    def normalize(self, angle):
            if math.fabs(angle) > math.pi:
                angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
            return angle

    def go_to(self, goal: list):

        cmd_vel_msg = Twist()
        distance_x = goal[0][0] - self.robotPos[0]
        distance_y = goal[0][1] - self.robotPos[1]
        theta = math.atan2((distance_y), (distance_x))
        distance = math.sqrt(distance_x ** 2 + distance_y ** 2)
        if self.estado == 0:
            if (self.tempo_anterior + 0.1 <= self.tempo_atual):
                if (min(self.ranges)>=0.28):
                    self.tempo_anterior = self.tempo_atual
                    self.points.append(self.robotPos)
            coeficiente_estado = 1
            front_distance = min(self.zone_F)
            minimo_dist = 0.3   
        else:
            coeficiente_estado = -1
            minimo_dist = 0.3
            front_distance = min(self.zone_B)
            theta = theta - ((1 * math.pi) if theta>=0 else (-1*math.pi))
        

        yaw_diff = self.normalize(theta - self.yaw)

        if goal[1] * coeficiente_estado == 1:
            current_side = -1 * coeficiente_estado
            side = min(self.zone_R)
            counter_side = min(self.zone_L)
        else:
            current_side = 1 * coeficiente_estado
            side = min(self.zone_L)
            counter_side = min(self.zone_R)
        
        if abs(distance_x) > minimo_dist or abs(distance_y) > minimo_dist:
            if front_distance < 0.28:
                cmd_vel_msg.linear.x = 0.01
                cmd_vel_msg.angular.z = 0.5 * (-1.0 * current_side) * coeficiente_estado
            elif side < 0.28:
                cmd_vel_msg.linear.x = (float(side) * 2) if side > 0.12 else 0.1
                cmd_vel_msg.angular.z = .5 * (-1.0 * current_side) * coeficiente_estado
            elif counter_side < 0.28:
                cmd_vel_msg.linear.x = 0.1
                cmd_vel_msg.angular.z = .5 * (current_side) * coeficiente_estado
            else:
                cmd_vel_msg.linear.x = .5 * min(front_distance, distance) if min(front_distance, distance) < 0.7 else 0.5
                cmd_vel_msg.angular.z = 1 * (yaw_diff)

            if self.estado == 1:
                cmd_vel_msg.linear.x = -1 * cmd_vel_msg.linear.x
            
            self.publisher.publish(cmd_vel_msg)
            

        elif (self.minRange + 0.02 > min(self.ranges)):
            cmd_vel_msg.linear.x = -0.01
            cmd_vel_msg.angular.z = .25 * (-1.0 * current_side)
            self.publisher.publish(cmd_vel_msg)

        else:
            if self.estado == 0:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.publisher.publish(cmd_vel_msg)
                self.estado = 1
                time.sleep(2)
            else:
                self.j = self.j+1

            return True
        return False

def main(args=None):
    rclpy.init(args=None)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
