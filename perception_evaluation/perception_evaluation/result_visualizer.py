#!/usr/bin/env python3

import os

import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

from sklearn.cluster import DBSCAN

import numpy as np
import pandas as pd

class ResultViz():
    def __init__(self, csv_name):
        self.result_path = "/home/kiapi/autoware_dep_ws/src/perception_evaluation/result"
        self.csv_path = os.path.join(self.result_path, csv_name)
        self.global_index = 0
        self.obs_list = []

        pd_csv = pd.read_csv(self.csv_path)
        self.ParsingDF(pd_csv, csv_name)
        plt.show()

    def ParsingDF(self, pd_csv, csv_name):
        label_filter = 1
        df_filtered = pd_csv[pd_csv['label'] == label_filter]

        coords = df_filtered[['pose_x', 'pose_y']].to_numpy()
        clustering = DBSCAN(eps=2.75, min_samples=3).fit(coords)
        df_filtered['cluster_id'] = clustering.labels_

        # print(clustering.labels_)
        
        for cluster_id, group in df_filtered.groupby('cluster_id'):
            print(cluster_id)
            df_list = []
            if cluster_id == -1:
                continue  # Skip noise
            

            
            stamp = group['stamp'].to_numpy().flatten()
            x = group['pose_x'].to_numpy().flatten()
            y = group['pose_y'].to_numpy().flatten()
            z = group['pose_z'].to_numpy().flatten()
            
            qx = group['ori_x'].to_numpy().flatten()
            qy = group['ori_y'].to_numpy().flatten()
            qz = group['ori_z'].to_numpy().flatten()
            qw = group['ori_w'].to_numpy().flatten()

            df_csv = pd.DataFrame({
            'stamp': stamp,
            'x': x,
            'y': y,
            'z': z,
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw
            })

            

            df_name = csv_name + "cluster_" + str(cluster_id) + ".txt"
            df_csv.to_csv(os.path.join(self.result_path, df_name), sep=' ', index=False, header=False)
            plt.plot(x, y, label=f'Cluster {cluster_id}')
        

        plt.legend()
        plt.title("Clusters of Annotated Items (Label = 1)")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.axis("equal")
        plt.show()
        

if __name__ == '__main__':
    # node = ResultViz('centerpoint.csv')
    # node = ResultViz('centerpoint_tiny.csv')
    # node = ResultViz('transfusion.csv')
    node = ResultViz('apollo.csv')
