#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32
from std_msgs.msg import Float32



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('number_subscriber')

        # subscriber for 'number' topic
        self.subscription = self.create_subscription(
            UInt32, 'number', self.listener_callback, 10)
        self.subscription               # prevent unused variable warning

        # publisher for elapsed time
        self.diff_publisher = self.create_publisher(Float32, 'diff', 10)
        self.timepoint = time.time()    # object to save timepoint of previous message


    def listener_callback(self, msg):
        # measure and publish elapsed time
        now = time.time()
        diff = Float32()
        diff.data = now - self.timepoint
        self.diff_publisher.publish(diff)
        self.timepoint = now

        self.get_logger().info('Received: "%i" after %.5f s' % (msg.data, diff.data))



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    