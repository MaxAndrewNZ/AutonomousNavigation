import sys
sys.path.insert(1, '../side_camera_test/')
import time,zmq,pickle
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
import vehicle as veh
import pointcloud as pc
import visualise_pointclouds as visualise
import cv2
import math
from matplotlib import pyplot as plt
import matplotlib.lines as mlines
from matplotlib import cm
import random
import os


def main():
    print("List Original")
    a = np.array([[1, 2, 3], 
                [2, 2, 3], 
                [2, 1, 3], 
                [1, 1, 2], 
                [0, 0, 1]])
    print(a)

    print("List delete column")
    b = np.delete(a, 1, axis=1)
    print(b)

    print("List remove duplicates")
    c = np.unique(b, axis=0)
    print(c)

main()