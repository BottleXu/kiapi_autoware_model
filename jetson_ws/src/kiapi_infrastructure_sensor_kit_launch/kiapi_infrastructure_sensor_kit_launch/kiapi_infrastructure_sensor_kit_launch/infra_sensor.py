#!/usr/bin/env python3

import rclpy
import yaml
import os

from ament_index_python.packages import get_package_share_directory

# import kiapi_infrastructure_sensor_kit_launch.transformations as transformations
from tf_transformations import quaternion_from_euler

from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class InfraSensor(Node):
    def __init__(self):
        super().__init__('kiapi_InfraSensor')

        self.declare_parameter('mode', 'topic') # topic or tf
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.get_logger().info('kiapi_InfraSensor-Running mode is: %s' % self.mode)


        self._pub_imu = self.create_publisher(Imu, 'imu/data_zero', 10)
        self._pub_gnss = self.create_publisher(NavSatFix, 'gnss/fixed_zero', 10)
        self._pub_vel_info = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.msg_imu = Imu()
        self.msg_gnss = NavSatFix()
        self.msg_tf = TransformStamped()
        self.vel_info_msg = VelocityReport()

        pkg_dir = get_package_share_directory("kiapi_infrastructure_sensor_kit_launch")
        # node_dir = os.getcwd()
        # pkg_dir, _ = os.path.split(node_dir)
        # print(pkg_dir)
        config_path = os.path.join(pkg_dir, "config", "infra_node.param.yaml")      
        
        self.InitValue()
        self.PasringYaml(config_path)

        if self.mode == 'tf':
            self.create_timer(1.0 / self.pub_rate, self.TFCallback)
        else:
            self.create_timer(1.0 / self.pub_rate, self.TimerCallback)

    def PasringYaml(self, yaml_path):
        with open(yaml_path, "r") as f:
            p = yaml.safe_load(f)["/**"]["ros__parameters"]

        self.pub_rate = float(p["rate"])
        self.msg_imu.header.frame_id = p["imu"]["frame"]
        imu_heading_yaw = float(p["imu"]["heading_yaw"])

        q = quaternion_from_euler(ai=0, aj=0, ak=imu_heading_yaw)
        self.msg_imu.orientation.x = q[0]
        self.msg_imu.orientation.y = q[1]
        self.msg_imu.orientation.z = q[2]
        self.msg_imu.orientation.w = q[3]

        self.msg_gnss.header.frame_id = p["gnss"]["frame"]
        self.msg_gnss.latitude = float(p["gnss"]["latitude"])
        self.msg_gnss.longitude = float(p["gnss"]["longitude"])
        self.msg_gnss.altitude = float(p["gnss"]["altitude"])

        self.vel_info_msg.heading_rate = imu_heading_yaw
        
        self.msg_tf.header.frame_id = 'map'
        self.msg_tf.child_frame_id = 'base_link'
        self.msg_tf.transform.translation.x = float(p["direct_tf"]["translation_x"])
        self.msg_tf.transform.translation.y = float(p["direct_tf"]["translation_y"])
        self.msg_tf.transform.translation.z = float(p["direct_tf"]["translation_z"])
        self.msg_tf.transform.rotation.x = float(p["direct_tf"]["rotation_x"])
        self.msg_tf.transform.rotation.y = float(p["direct_tf"]["rotation_y"])
        self.msg_tf.transform.rotation.z = float(p["direct_tf"]["rotation_z"])
        self.msg_tf.transform.rotation.w = float(p["direct_tf"]["rotation_w"])

    
    def InitValue(self):
        self.msg_gnss.status.status = 2
        self.msg_gnss.status.service = 3

        self.msg_gnss.position_covariance_type = 2
        self.vel_info_msg.header.frame_id = 'base_link'

    def TimerCallback(self):
        now = self.get_clock().now().to_msg()
        self.msg_imu.header.stamp = now
        self.msg_gnss.header.stamp = now
        self.vel_info_msg.header.stamp = now

        self._pub_imu.publish(self.msg_imu)
        self._pub_gnss.publish(self.msg_gnss)
        self._pub_vel_info.publish(self.vel_info_msg)

    def TFCallback(self):
        self.msg_tf.header.stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(self.msg_tf)
        # self.get_logger().info('TFPub from %s to %s' % self.msg_tf.header.frame_id % self.msg_tf.child_frame_id)


def main(args=None):
    rclpy.init(args=args)
    node = InfraSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(Exception)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()