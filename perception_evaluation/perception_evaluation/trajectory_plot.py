#!/usr/bin/env python3

import os
import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sklearn.metrics import mean_squared_error


# ['stamp','x','y','z','qx','qy','qz','qw'])
class XYPose():
    def __init__(self, txt_path):
        # print(txt_path)
        self.df = pd.read_csv(txt_path, sep=' ', header=None)
        self.df.columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

        # print(txt_path)
        # print(self.df.head(2))

        print(self.df.head(4))

        self.CalcDeltaXY2(txt_path)



    def CalcDeltaXY(self, txt_path):
        div_i = 0
        self.df['delta_x'] = (self.df['x'].diff())**2    # square
        self.df['delta_y'] = (self.df['y'].diff())**2   
        for idx, row in self.df.iterrows():

            if -50 < row['x'] < 50:     #range in +- 50 m
                div_i += 1
            else:
                self.df.loc[idx, 'delta_x'] = 0.0
                self.df.loc[idx, 'delta_y'] = 0.0
        # print(div_i)
        # print(self.df.head(5))
        
        # Sum of squares
        s_sum_x = self.df['delta_x'].sum()
        s_sum_y = self.df['delta_y'].sum()
        ms_x = s_sum_x / div_i
        ms_y = s_sum_y / div_i
        rms_x = math.sqrt(ms_x)
        rms_y = math.sqrt(ms_y)
        print(len(self.df))
        print(f"X = {s_sum_x}, ms = {ms_x}, rms={rms_x}")
        print(f"Y = {s_sum_y}, ms = {ms_y}, rms={rms_y}")

        self.df['delta_xy'] = self.df['delta_x'] + self.df['delta_y']
        rms_xy = math.sqrt(self.df['delta_xy'].mean())
        print(f"XY RMS distance = {rms_xy:.4f}")

        # df_name = txt_path + "cluster_concat.txt"
        # self.df.to_csv(df_name, sep=' ', index=False, header=False)
    
    def CalcDeltaXY2(self, txt_path):
        # Calculate squared deltas
        self.df['delta_x'] = self.df['x'].diff().fillna(0) ** 2
        self.df['delta_y'] = self.df['y'].diff().fillna(0) ** 2

        # Mask for points within ±50m of X
        mask = self.df['x'].between(-50, 50)
        div_i = mask.sum()

        # Zero out values outside range
        self.df.loc[~mask, ['delta_x', 'delta_y']] = 0.0
            
        # RMS computation
        s_sum_x = self.df['delta_x'].sum()
        s_sum_y = self.df['delta_y'].sum()
        ms_x = s_sum_x / div_i
        ms_y = s_sum_y / div_i
        rms_x = math.sqrt(ms_x)
        rms_y = math.sqrt(ms_y)

        self.df['delta_xy'] = self.df['delta_x'] + self.df['delta_y']
        rms_xy = math.sqrt(self.df['delta_xy'].mean())

        print(f"{txt_path}")
        print(f"  Total points: {len(self.df)}, In-range: {div_i}")
        print(f"  X RMS = {rms_x:.7f}, Y RMS = {rms_y:.7f}, XY RMS = {rms_xy:.7f}")


class XYConcatPose():
    def __init__(self, dir, f_list):
        for idx_i, file_i in enumerate(f_list):
            
            txt_path = os.path.join(dir, file_i)
            # print(idx_i, file_i)

            df_i = pd.read_csv(txt_path, sep=' ', header=None)
            df_i.columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

            if idx_i == 0:
                self.df = df_i
            else:
                self.df = pd.concat([self.df, df_i], ignore_index=True)
                
        # print(f_list[0])
        print(self.df.head(2))

        # df_name = f_list[0] + "cluster_concat.txt"
        # self.df.to_csv(os.path.join(dir, df_name), sep=' ', index=False, header=False)

        self.CalcDeltaXY2(txt_path)
    
    def CalcDeltaXY2(self, txt_path):
        # Calculate squared deltas
        self.df['delta_x'] = self.df['x'].diff().fillna(0) ** 2
        self.df['delta_y'] = self.df['y'].diff().fillna(0) ** 2

        # Mask for points within ±50m of X
        mask = self.df['x'].between(-50, 50)
        div_i = mask.sum()

        # Zero out values outside range
        self.df.loc[~mask, ['delta_x', 'delta_y']] = 0.0
            
        # RMS computation
        s_sum_x = self.df['delta_x'].sum()
        s_sum_y = self.df['delta_y'].sum()
        ms_x = s_sum_x / div_i
        ms_y = s_sum_y / div_i
        rms_x = math.sqrt(ms_x)
        rms_y = math.sqrt(ms_y)

        self.df['delta_xy'] = self.df['delta_x'] + self.df['delta_y']
        rms_xy = math.sqrt(self.df['delta_xy'].mean())

        print(f"{txt_path}")
        print(f"  Total points: {len(self.df)}, In-range: {div_i}")
        print(f"  X RMS = {rms_x:.7f}, Y RMS = {rms_y:.7f}, XY RMS = {rms_xy:.7f}")


            
class TrajectoryPlot():
    def __init__(self, dir_root, case):

        if case == 1:
            ### Case 1: rosbag2_2023-09-15-13-57-04_tracking
            dir = os.path.join(dir_root, "rosbag2_2023-09-15-13-57-04_tracking")
            ctp = "centerpoint_cluster_1.txt"
            ctp_tiny = "centerpoint_tiny_cluster_1.txt"
            trans = "transfusion_cluster_1.txt"
            aplo = "apollo_cluster_3.txt"
        elif case == 2:
            ### Case 2: rosbag2_2023-09-15-14-00-19_tracking
            dir = os.path.join(dir_root, "rosbag2_2023-09-15-14-00-19_tracking")
            ctp = "centerpoint_cluster_1.txt"
            ctp_tiny = "centerpoint_tiny_cluster_1.txt"
            trans = "transfusion_cluster_1.txt"
            aplo = "apollo_cluster_3.txt"
        elif case == 3:
            ### Case 3: rosbag2_2023-09-15-14-09-01_tracking
            dir = os.path.join(dir_root, "rosbag2_2023-09-15-14-09-01_tracking")
            ctp = "centerpoint_cluster_1.txt"
            ctp_tiny = "centerpoint_tiny_cluster_2.txt"
            trans = "transfusion_cluster_3.txt"
            aplo = "apollo_cluster_4.txt"
        elif case == 4:
            ### Case 4: rosbag2_2023-09-15-14-13-06_tracking
            dir = os.path.join(dir_root, "rosbag2_2023-09-15-14-13-06_tracking")
            ctp = ["centerpoint_cluster_1.txt", "centerpoint_cluster_2.txt", "centerpoint_cluster_3.txt", "centerpoint_cluster_4.txt"]
            ctp_tiny = ["centerpoint_tiny_cluster_1.txt", "centerpoint_tiny_cluster_2.txt", "centerpoint_tiny_cluster_3.txt"]
            trans = ["transfusion_cluster_1.txt", "transfusion_cluster_2.txt"]
            aplo = ["apollo_cluster_3.txt", "apollo_cluster_6.txt", "apollo_cluster_7.txt", "apollo_cluster_8.txt"]
        else:
            return

        if case == 4:
            self.centerpoint = XYConcatPose(dir, ctp)
            self.centerpoint_tiny = XYConcatPose(dir, ctp_tiny)
            self.transfusion = XYConcatPose(dir, trans)
            self.apollo = XYConcatPose(dir, aplo)

        else:
            centerpoint_txt = os.path.join(dir, ctp)
            centerpoint_tiny_txt = os.path.join(dir, ctp_tiny)
            transfusion_txt = os.path.join(dir, trans)
            apollo_txt = os.path.join(dir, aplo)

            self.centerpoint = XYPose(centerpoint_txt)
            self.centerpoint_tiny = XYPose(centerpoint_tiny_txt)
            self.transfusion = XYPose(transfusion_txt)
            self.apollo = XYPose(apollo_txt)
        
    def Plotdata(self):

        plt.figure(figsize=(15,9))

        p = plt.subplot(3, 1, 1)
        p.plot(self.centerpoint.df['x'], self.centerpoint.df['y'], label=f'centerpoint')
        p.plot(self.centerpoint_tiny.df['x'], self.centerpoint_tiny.df['y'], label=f'centerpoint_tiny')
        p.plot(self.transfusion.df['x'], self.transfusion.df['y'], label=f'transfusion')
        p.plot(self.apollo.df['x'], self.apollo.df['y'], label=f'apollo')
        p.set_xlabel('x (m)')
        p.set_ylabel('y (m)')
        p.set_title('position (x,y) per time')
        p.invert_xaxis()

        p = plt.subplot(3, 1, 2)
        p.plot(self.centerpoint.df['timestamp'], self.centerpoint.df['x'], label=f'centerpoint')
        p.plot(self.centerpoint_tiny.df['timestamp'], self.centerpoint_tiny.df['x'], label=f'centerpoint_tiny')
        p.plot(self.transfusion.df['timestamp'], self.transfusion.df['x'], label=f'transfusion')
        p.plot(self.apollo.df['timestamp'], self.apollo.df['x'], label=f'apollo')
        p.set_xlabel('t (s)')
        p.set_ylabel('x (m)')
        p.set_title('x per time')
        # p.invert_yaxis()

        p = plt.subplot(3, 1, 3)
        p.plot(self.centerpoint.df['timestamp'], self.centerpoint.df['y'], label=f'centerpoint')
        p.plot(self.centerpoint_tiny.df['timestamp'], self.centerpoint_tiny.df['y'], label=f'centerpoint_tiny')
        p.plot(self.transfusion.df['timestamp'], self.transfusion.df['y'], label=f'transfusion')
        p.plot(self.apollo.df['timestamp'], self.apollo.df['y'], label=f'apollo')
        p.set_xlabel('t (s)')
        p.set_ylabel('y (m)')
        p.set_title('y per time')
        # p.invert_yaxis()


        

        

        # plt.scatter(self.centerpoint.df['x'], self.centerpoint.df['y'], label=f'centerpoint')
        # plt.scatter(self.centerpoint_tiny.df['x'], self.centerpoint_tiny.df['y'], label=f'centerpoint_tiny')
        # plt.scatter(self.transfusion.df['x'], self.transfusion.df['y'], label=f'transfusion')
        # plt.scatter(self.apollo.df['x'], self.apollo.df['y'], label=f'apollo')

        # plt.Axes()
        # plt.gca().invert_xaxis()
        plt.tight_layout()
        plt.legend()
        plt.show()

if __name__ == '__main__':
    dir_root = "~/autoware_dep_ws/src/perception_evaluation/result"
    cp = TrajectoryPlot(dir_root, 4)
    cp.Plotdata()