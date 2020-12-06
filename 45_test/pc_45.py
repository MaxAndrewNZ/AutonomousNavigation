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

def npToPcd(np):
    """
    Converts a numpy array to an Open3D pointcloud object. 

    Args:
        np: A numpy array of a pointcloud.

    Returns:
        Pointcloud: An Open3D pointcloud object.
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np)

    return pcd

def get_minimal_distance(pointcloud):
    npCloud = np.asarray(pointcloud.pcd.points)
    min_dist_index = np.argmin(npCloud[:,2], axis=0)
    min_dist_point = npCloud[min_dist_index]
    print("Min point:", min_dist_point)
    min_dist = math.sqrt((min_dist_point[0] ** 2 + min_dist_point[2] ** 2))

    return min_dist_point, min_dist

def display_errors(pointcloud, closest_point, region_min, region_max, linear_errors, angular_errors):
    #TODO: Possibly plot the target angle and distance on the graphs
    np_cloud = np.asarray(pointcloud.pcd.points)
    plt.ion()
    plt.show()
    plt.clf()

    plt.subplot(3, 1, 1)
    plt.scatter(np_cloud[:,0], np_cloud[:,2], color='grey')
    plt.scatter(closest_point[0], closest_point[1], color='red')
    plt.plot([0, closest_point[0]], [0, closest_point[1]], color='red')
    plt.xlim(region_min[0], region_max[0])
    plt.ylim(region_min[2], region_max[2])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X distance')
    plt.ylabel('Z distance')
    plt.title('Top down view - Closest Point')

    plt.subplot(3, 1, 2)
    plt.plot(*zip(*linear_errors), color='blue')
    plt.ylabel('Linear Error')

    plt.subplot(3, 1, 3)
    plt.plot(*zip(*angular_errors), color='blue')
    plt.xlabel('time (seconds)')
    plt.ylabel('Angular Error')

    plt.draw()
    plt.pause(0.001)

def plot_errors(linear_errors, angular_errors):
    plt.subplot(2, 1, 1)
    plt.plot(*zip(*linear_errors))
    plt.ylabel('Linear Error')
    plt.title('Linear and angular error over time')

    plt.subplot(2, 1, 2)
    plt.plot(*zip(*angular_errors))
    plt.xlabel('time (seconds)')
    plt.ylabel('Angular Error')

    plt.show()

def set_velocity(min_distance, orientation, target_distance, max_speed):
    # Velocities
    vel_stop = 0
    vel_mid = max_speed * 0.5
    vel_slow = max_speed * 0.4
    vel_full = max_speed

    if min_distance < target_distance:
        velocity = vel_stop
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

def k_nearest(flat_map):
    #TODO: Implement k nearest
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
    testing = False
    visualising = False
    plotting_error = True
    save_clouds = False
    custom_folder_name = "testing_pc_45"

    following_side = "left" # left or right
    target_distance = 0.4 # Meters
    error_distance = 0.1 # Meters
    error_angle = 10.0 # Degrees

    speed = 0.3 # Speed of the vehicle
    turn_speed = 0.5 # Speed * 2 normally
    turn_time_multiplier = 0.0 # Consecutive turns get bigger
    maximum_turns = 10 # Vehicle will stop after turning this many times in a row
    min_points_for_avoidance = 80 # Increase if navigation is disrupted due to noise

    max_speed = 1
    angular_limit = 2.5

    # Visualisation region colours37
    main_colour = [0.45, 0.45, 0.45]

    # Regions
    num_regions = 5
    angle_range = 180
    x_max = 365 #TODO: Update this to the actual size
    region_size = x_max / 5

    close_dist = 0.5
    med_dist = 1.0
    inf_dist = 2.0
    distance_limits = [close_dist, med_dist, inf_dist]

    # Proportional and derivative constants
    p_const = 15
    d_const = 0

    linear_errors = []
    angular_errors = []

    # Region of interest
    minX = -3
    maxX = 3

    minY = -1.5
    maxY = -0.25

    minZ = 0.05
    maxZ = 3

    region_min = [minX, minY, minZ]
    region_max = [maxX, maxY, maxZ]

    ############      main loop        ##################
    vehicle = veh.Vehicle("tcp://192.168.8.106", "5556", "5557")

    if testing == False:
        vehicle.connect_control()
        print('Connected to vehicle server')

    vehicle.set_min_points_for_avoidance(min_points_for_avoidance) 
    vehicle.set_following_variables(following_side, target_distance, error_distance, error_angle)
    vehicle.set_control_variables(speed, turn_speed, turn_time_multiplier, maximum_turns)

    found_cloud = False
    updated_cloud = False

    custom_folder_name += "_" + following_side + "_" + str(target_distance) + "_" + str(speed) + "_" + str(int(time.time()))
    time_start = time.time()
    count = 0.1
    try:
        while 1:
            if (testing):
                # Use saved pointcloud file
                filename = "1.ply"
                pcd = o3d.io.read_point_cloud(filename)
                npCloud = np.asarray(pcd.points)
                # Flips points to align with those from the landrov
                offset = np.array([-1.8 + count, 0, -0.5 - count])
                pcd = pc.npToPcd(npCloud * np.array([1, -1, -1]) + offset)
                # Simulate delays in recieving pointcloud
                time.sleep(0.1)
                count += 0.1
                found_cloud = True
       
            if not found_cloud:
                vehicle.connect_pointcloud()
                while not found_cloud:
                    if  len(zmq.select([vehicle.sensor_socket],[],[],0)[0]):
                        topic,buf = vehicle.sensor_socket.recv_multipart()
                        if topic == b'pointcloud':
                            time_start = time.time()
                            np_pcd = np.fromstring(buf, dtype=np.float32)
                            num_points = np_pcd.size // 3
                            reshaped_pcd = np.resize(np_pcd, (num_points, 3))
                            pcd = o3d.geometry.PointCloud()
                            pcd.points = o3d.utility.Vector3dVector(reshaped_pcd)
                            found_cloud = True

                vehicle.sensor_socket.close()

            else:
                downpcd = pcd.voxel_down_sample(voxel_size=0.04)

                cloud = pc.PointCloud(downpcd, main_colour, region_min, region_max)
                cloud.pcd, ind = cloud.pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

                if len(cloud.pcd.points) == 0:
                    break

                flat_cloud = flatten_cloud(cloud)
                min_distance_point, min_dist = k_nearest(flat_cloud)

                # Print status
                print("*" * 64)
                template = "Points in cloud {}"

                print(template.format(len(cloud.pcd.points)))
                print("Time taken", round(time.time() - time_start, 2))
                time_start = time.time()

                # Navigation
                # TODO: Remove 3D nearest distance calculation
                # min_distance_point, min_dist = get_minimal_distance(cloud)
                current_time = time.time()
                min_dist_angle = math.atan((min_distance_point[0] / min_distance_point[1]))
                print("Min Distance:", round(min_dist, 2))
                print("Angle Rad:", round(min_dist_angle, 2))
                print("Angle Degrees:", round(math.degrees(min_dist_angle), 2))
                linear_velocity = set_velocity(min_dist, min_dist_angle, target_distance, max_speed)

                # Wall distance error
                d_error = min_dist - target_distance
                if len(linear_errors) == 0:
                    linear_errors.append((current_time - 1, 0))
                linear_pd = (p_const * d_error) + d_const * ((d_error - linear_errors[-1][1]) / (current_time - linear_errors[-1][0]))
                linear_errors.append((current_time, d_error))

                # Wall angle error
                #TODO: Check if this should be + or -
                a_error = min_dist_angle + (math.pi / 2)
                print("A error:", round(a_error, 2), "D error:", round(d_error, 2))
                angular_p = p_const * a_error
                angular_errors.append((current_time, a_error))

                # Calculate angular velocity
                angular_velocity = max(min(linear_pd + angular_p, angular_limit), - angular_limit)

                print("Linear velocity:", linear_velocity)
                print("Angular velocity:", angular_velocity)

                if (visualising):
                    closest_point_cloud = npToPcd([min_distance_point])
                    points = [[0,0,0], min_distance_point]
                    lines = [[0,1]]
                    colors = [[1, 0, 0] for i in range(len(lines))]
                    line_set = o3d.geometry.LineSet()
                    line_set.points = o3d.utility.Vector3dVector(points)
                    line_set.lines = o3d.utility.Vector2iVector(lines)
                    line_set.colors = o3d.utility.Vector3dVector(colors)
                    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
                    # print(cloud.pcd[outliers])
                    outlier_cloud = cloud.pcd.select_down_sample(ind, invert=True)
                    # outlier_cloud.colors = [1, 0, 0]
                    outlier_cloud.paint_uniform_color([1, 0, 0])

                    visualise.visualise([cloud.pcd, mesh_frame, closest_point_cloud, line_set, outlier_cloud])
                    time.sleep(0.5)

                if plotting_error:
                    display_errors(cloud, min_distance_point, region_min, region_max, linear_errors, angular_errors)

                command = "forward"

            if updated_cloud and not testing:
                if command == "stop":
                    print("Landrov cannot find clear path...")
                    print("Stopping")
                    vehicle.stop()
                    break
                elif command == "forward":
                    vehicle.reset_turn_counts()
                    vehicle.velocity_to_motor_command(linear_velocity, angular_velocity)
                else:
                    print("No Command")
                    vehicle.stop()
                
                found_cloud = False
                updated_cloud = False

    except KeyboardInterrupt:
        print("Force Close")
        if not testing:
            vehicle.stop()
        if plotting_error:
            plot_errors(linear_errors, angular_errors)

main()