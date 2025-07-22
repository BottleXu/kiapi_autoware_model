#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PCLSync(Node):
    def __init__(self):
        super().__init__('kiapi_hesai_sync')

        print("Node Init")

        self._pub_pc2 = self.create_publisher(PointCloud2, 'pointcloud_raw_synced', 10)
        self._sub_pc2 = self.create_subscription(PointCloud2, 'pointcloud_raw_ex', self.PC2Callback,10)


    
    def PC2Callback(self, msg):
        PointCloud2_msg = msg
        PointCloud2_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_pc2.publish(PointCloud2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PCLSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(Exception)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()