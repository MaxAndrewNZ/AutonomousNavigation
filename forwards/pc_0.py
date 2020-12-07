import sys
sys.path.insert(1, '../side_camera_test/')
import time,zmq,pickle
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
import vehicle as veh
import pointcloud as pc
import visualise_pointclouds as visualise
import display_information as display
import cv2
import math
import random
import os


def fit_line_to_points(points):
    X = points[:,0]
    Y = points[:,1]

    xbar = sum(X)/len(X)
    ybar = sum(Y)/len(Y)
    n = len(X) # or len(Y)

    numer = sum([xi*yi for xi,yi in zip(X, Y)]) - n * xbar * ybar
    denum = sum([xi**2 for xi in X]) - n * xbar**2

    b = numer / denum
    a = ybar - b * xbar

    print('best fit line:\ny = {:.2f} + {:.2f}x'.format(a, b))

    return a, b


def get_test_pointcloud(moving, shift_x, shift_y, delay=0.3):
    # Use saved pointcloud file
    filename = "1.ply"
    pcd = o3d.io.read_point_cloud(filename)
    npCloud = np.asarray(pcd.points)
    # Flips points to align with those from the landrov
    offset = np.array([-1.8 + shift_x, 0, -0.5 - shift_y])
    pcd = pc.npToPcd(npCloud * np.array([1, -1, -1]) + offset)
    # Simulate delays in recieving pointcloud
    time.sleep(delay)
    if moving:
        shift_x += 0.05 * random.randint(0, 3)
        shift_y += 0.05 * random.randint(0, 3)
    
    return pcd, shift_x, shift_y

def set_velocity(min_distance, orientation, target_distance, max_speed):
    vel_stop = 0
    vel_mid = max_speed * 0.5
    vel_slow = max_speed * 0.4
    vel_full = max_speed

    if min_distance < target_distance * 0.5:
        velocity = vel_stop
    elif min_distance < target_distance:
        velocity = vel_slow
    elif min_distance < 2 * target_distance:
        velocity = vel_mid
    elif orientation > 1.75:
        print("Large difference in angle.")
        velocity = vel_slow
    else:
        velocity = vel_full

    return velocity

def flatten_cloud(cloud):
    np_cloud = np.asarray(cloud.pcd.points)
    removed_y = np.delete(np_cloud, 1, axis=1)
    removed_dups = np.unique(removed_y, axis=0)

    return removed_dups

def get_closest_point(flat_map):
    dist_2 = np.sum((flat_map - [0, 0])**2, axis=1)
    closest_point = np.argmin(dist_2)
    print(closest_point)
    distance = math.sqrt(flat_map[closest_point][0]**2 + flat_map[closest_point][1]**2)

    return flat_map[closest_point], distance

def main():
    """   
    This method is the heart of the Landrov navigation. 
    It handles controling the multi-step navigation. 
    The navigation settings can be configured within this.
    """
  
    ############      Configuration        ##################
    testing = True
    plotting_error = True
    moving = True

    target_distance = 1.0 # Meters
    max_speed = 1.0 # 0 - 1

    # Proportional and derivative constants
    linear_p_const = 0
    angular_p_const = 1
    # Data lists
    linear_errors = []
    angular_errors = []
    linear_velocities = []
    angular_velocities = []

    # Region of interest
    minX = -2.5
    maxX = 0

    minY = -1.5
    maxY = -0.25

    minZ = 0.01
    maxZ = 3.0

    region_min = [minX, minY, minZ]
    region_max = [maxX, maxY, maxZ]

    vehicle = veh.Vehicle("tcp://192.168.8.106", "5556", "5557")

    if testing == False:
        vehicle.connect_control()
        print('Connected to vehicle server')

    found_cloud = False
    updated_cloud = False

    time_start = time.time()

    shift_x = 0.1
    shift_y = 0.1

    try:
        while 1:
            time_start = time.time()
            if (testing):
                pcd, shift_x, shift_y = get_test_pointcloud(moving, shift_x, shift_y)
                time_start = time.time()
                found_cloud = True
       
            if not found_cloud:
                vehicle.connect_pointcloud()
                while not found_cloud:
                    if len(zmq.select([vehicle.sensor_socket],[],[],0)[0]):
                        topic,buf = vehicle.sensor_socket.recv_multipart()
                        if topic == b'pointcloud':
                            np_pcd = np.fromstring(buf, dtype=np.float32)
                            num_points = np_pcd.size // 3
                            reshaped_pcd = np.resize(np_pcd, (num_points, 3))
                            pcd = o3d.geometry.PointCloud()
                            pcd.points = o3d.utility.Vector3dVector(reshaped_pcd)
                            print("Time get cloud", round(time.time() - time_start, 2))
                            time_start = time.time()
                            found_cloud = True

                vehicle.sensor_socket.close()

            else:
                downpcd = pcd.voxel_down_sample(voxel_size=0.1)
                cloud = pc.PointCloud(downpcd, [0, 0, 0], region_min, region_max)
                cloud.pcd, ind = cloud.pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                updated_cloud = True

                if len(cloud.pcd.points) == 0:
                    break

                flat_cloud = flatten_cloud(cloud)

                # Print status
                print("*" * 64)
                template = "Points in cloud {}"

                print(template.format(len(cloud.pcd.points)))

                #TODO: Fit line to cloud.
                a, b = fit_line_to_points(flat_cloud)
                #TODO: Calculate angle error and distance error.
                angle = - (math.atan(b) + (math.pi / 2))
                a_error = angle
                angular_p = angular_p_const * a_error

                distance = abs(a / (math.sqrt((b ** 2 + 1))))
                d_error = distance - target_distance

                current_time = time.time()

                if len(linear_errors) == 0:
                    linear_errors.append((current_time - 1, 0))
                linear_p = linear_p_const * d_error
                linear_errors.append((current_time, d_error))

                # Wall angle error
                angular_errors.append((current_time, a_error))

                #TODO: Make a navigation adjustment from this information.
                angular_velocity = max(min(angular_p + linear_p, 2), -2)
                linear_velocity = max_speed * 0.3

                print("Target Distance:", target_distance, "Distance:", round(distance, 2), "D error:", round(d_error, 2))
                print("Angle Degrees:", round(math.degrees(angle), 2), "Angle Rad:", round(angle, 2), "A error:", round(a_error, 2))
                print("Linear velocity:", round(linear_velocity, 2), "Angular velocity:", round(angular_velocity, 2))
                       
                linear_velocities.append((current_time, linear_velocity))
                angular_velocities.append((current_time, angular_velocity))

                # Plot this information
                if plotting_error:
                    display.display_line_follow(flat_cloud, a, b, region_min, region_max, linear_errors, angular_errors, linear_velocities, angular_velocities)
                
            if updated_cloud:
                print("Time processing", round(time.time() - time_start, 2))

            if updated_cloud and not testing:
                if moving:
                    vehicle.velocity_to_motor_command(linear_velocity, angular_velocity)
                
                found_cloud = False
                updated_cloud = False

    except KeyboardInterrupt:
        print("Force Close")
        if not testing:
            vehicle.stop()
        display.display_errors(linear_errors, angular_errors)

main()