#!/usr/bin/env python3

import os

import numpy as np
import pandas as pd
import csv

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from autoware_perception_msgs.msg import DetectedObjects, DetectedObject



class ObsfromPerc(Node):
    def __init__(self):
        super().__init__("autoware_perception_evaluation")
        
        self.label_dict = {
            0: "UNKNOWN",
            1: "CAR",
            2: "TRUCK",
            3: "BUS",
            4: "TRAILER",
            5: "MOTORCYCLE",
            6: "BICYCLE",
            7: "PEDESTRIAN",
            8: "ANIMAL",
            9: "HAZARD"
        }
        self.w_csv_data = []
        ### centerpoint; centerpoint/tiny
        # self.detection_model = 'centerpoint'
        ### transfusion
        # self.detection_model = 'transfusion'
        ### apollo
        self.detection_model = 'apollo'
        
        self.detection_topic = '/perception/object_recognition/detection/' + self.detection_model + '/objects'
        
        self.result_dir = "/home/kiapi/autoware_dep_ws/src/perception_evaluation/result"
        self.times_t = 0
        

        print(self.detection_topic)

        self._sub_obstacles = self.create_subscription(DetectedObjects, self.detection_topic, self.ObjectsCallback, 10)

        


    def ObjectsCallback(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        # print(stamp)
        for idx_i, item_i in enumerate(msg.objects):
            self.WriteCSV2(idx_i, item_i, stamp)
        self.times_t += 1


    def WriteCSV(self, idx_i, item_i, stamp_i):
        csv_name_l = [
            self.detection_model,
            "-",
            str(idx_i),
            ".csv"]
        # print(csv_name_l, type(csv_name_l))
        csv_name = "".join(csv_name_l)

        # print(csv_name)
        output_path = os.path.join(self.result_dir, csv_name)
        if not os.path.exists(output_path):
            with open(output_path, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'stamp',
                    'label',
                    'probability',
                    'pose_x',
                    'pose_y',
                    'pose_z',
                    'ori_x',
                    'ori_y',
                    'ori_z',
                    'ori_w'
                ])
                writer.writerow([
                    stamp_i,
                    item_i.classification[0].label,
                    item_i.classification[0].probability,
                    item_i.kinematics.pose_with_covariance.pose.position.x,
                    item_i.kinematics.pose_with_covariance.pose.position.y,
                    item_i.kinematics.pose_with_covariance.pose.position.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.x,
                    item_i.kinematics.pose_with_covariance.pose.orientation.y,
                    item_i.kinematics.pose_with_covariance.pose.orientation.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.w,
                ])

        else:
            with open(output_path, "a", newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    stamp_i,
                    item_i.classification[0].label,
                    item_i.classification[0].probability,
                    item_i.kinematics.pose_with_covariance.pose.position.x,
                    item_i.kinematics.pose_with_covariance.pose.position.y,
                    item_i.kinematics.pose_with_covariance.pose.position.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.x,
                    item_i.kinematics.pose_with_covariance.pose.orientation.y,
                    item_i.kinematics.pose_with_covariance.pose.orientation.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.w,
                ])

    def WriteCSV2(self, idx_i, item_i, stamp_i):
        csv_name_l = [
            self.detection_model,
            ".csv"]
        # print(csv_name_l, type(csv_name_l))
        csv_name = "".join(csv_name_l)

        # print(csv_name)
        output_path = os.path.join(self.result_dir, csv_name)
        if not os.path.exists(output_path):
            with open(output_path, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    't',
                    'stamp',
                    'index',
                    'label',
                    'probability',
                    'pose_x',
                    'pose_y',
                    'pose_z',
                    'ori_x',
                    'ori_y',
                    'ori_z',
                    'ori_w'
                ])
                writer.writerow([
                    self.times_t,
                    stamp_i,
                    idx_i,
                    item_i.classification[0].label,
                    item_i.classification[0].probability,
                    item_i.kinematics.pose_with_covariance.pose.position.x,
                    item_i.kinematics.pose_with_covariance.pose.position.y,
                    item_i.kinematics.pose_with_covariance.pose.position.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.x,
                    item_i.kinematics.pose_with_covariance.pose.orientation.y,
                    item_i.kinematics.pose_with_covariance.pose.orientation.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.w,
                ])
        else:
            with open(output_path, "a", newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.times_t,
                    stamp_i,
                    idx_i,
                    item_i.classification[0].label,
                    item_i.classification[0].probability,
                    item_i.kinematics.pose_with_covariance.pose.position.x,
                    item_i.kinematics.pose_with_covariance.pose.position.y,
                    item_i.kinematics.pose_with_covariance.pose.position.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.x,
                    item_i.kinematics.pose_with_covariance.pose.orientation.y,
                    item_i.kinematics.pose_with_covariance.pose.orientation.z,
                    item_i.kinematics.pose_with_covariance.pose.orientation.w,
                ])
        

def main(args=None):
    rclpy.init(args=args)
    node = ObsfromPerc()
    rclpy.spin(node)

    node.WriteCSV()


    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()