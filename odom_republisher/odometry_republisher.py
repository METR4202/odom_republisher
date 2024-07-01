#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Created by Kenzo Chooi and Jen Jen Chung

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf_transformations import euler_from_quaternion

class OdomRepublisher(Node):

    def __init__(self):
        super().__init__('odometry_republisher') # this defines the node name

        # Create a subscriber to the odom topic
        # The constructor inputs are (message type, topic name, associated callback function, message queue length)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher
        # The constructor inputs are (message type, topic name, message queue length)
        self.publisher_ = self.create_publisher(Twist, 'my_odom', 10)

    def odom_listener_callback(self, msg:Odometry):
        # Extract the (x,y,z) position data from the incoming odom message
        position = msg.pose.pose.position
        # Extract the (x,y,z,w) quaternion data from the incoming odom message
        orientation = msg.pose.pose.orientation

        # Print the (x,y) position to the command line along with the timestamp in nanoseconds
        self.get_logger().info('{}: (x: {}, y: {})'.format(msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec, 
                                                                  position.x, position.y))

        # Create a new Twist variable
        twist_msg = Twist()

        # Convert from quaternions to Euler angles using the tf_transformations library
        r, p, y = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Write the position and orientation data to the Twist variable
        twist_msg.linear.x = position.x
        twist_msg.linear.y = position.y
        twist_msg.linear.z = position.z
        twist_msg.angular.x = r
        twist_msg.angular.y = p
        twist_msg.angular.z = y

        # Publish on the my_odom topic
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create a new OdomRepublisher node
    odom_republisher = OdomRepublisher()

    # Execute the node
    rclpy.spin(odom_republisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()