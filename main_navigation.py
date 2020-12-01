import time,zmq,pickle
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
import pointcloud as pc
import vehicle as veh
import visualise_pointclouds as visualise
import math
import plane_estimation as pe
from ast import literal_eval
import random
import os


def main():
    """   
    This method is the heart of the Landrov navigation. 
    It handles controling the multi-step navigation. 
    The navigation settings can be configured within this.
    """
  
    ############      Configuration        ##################
    testing = True
    visualising = False
    save_clouds = False
    custom_folder_name = "testing"

    following_side = "left" # left or right
    target_distance = 1.0 # Meters
    error_distance = 0.25 # Meters
    error_angle = 7.0 # Degrees

    speed = 0.3 # Speed of the vehicle
    turn_speed = 0.5 # Speed * 2 normally
    turn_time_multiplier = 0.0 # Consecutive turns get bigger
    maximum_turns = 10 # Vehicle will stop after turning this many times in a row
    min_points_for_avoidance = 80 # Increase if navigation is disrupted due to noise

    # Pointcloud region configuration
    # X
    centerWidth = 0.5
    rightMinX = centerWidth / 2
    leftMaxX = - rightMinX

    maxWidth = 2.0
    rightMaxX = maxWidth / 2
    leftMinX = - rightMaxX

    # Z
    minZ = 0.2
    maxZ = 1.2

    stopMinZ = 0.0
    stopMaxZ = 0.5
    centerMaxZ = 1.5

    # Y
    minY = -0.5
    maxY = 0.17

    # Visualisation region colours
    stopColour = [1, 0, 0]
    centerColour = [0, 0, 1]
    leftColour = [0, 1, 0]
    rightColour = [0, 0, 0]

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

    # Regions (X, Y, Z)
    leftMin = [leftMinX, minY, minZ]
    leftMax = [leftMaxX, maxY, maxZ]

    stopMin = [leftMaxX, minY, stopMinZ]
    stopMax = [rightMinX, maxY, stopMaxZ]

    centerMin = [leftMaxX, minY, stopMaxZ]
    centerMax = [rightMinX, maxY, centerMaxZ]

    rightMin = [rightMinX, minY, minZ]
    rightMax = [rightMaxX, maxY, maxZ]

    custom_folder_name += "_" + following_side + "_" + str(target_distance) + "_" + str(speed) + "_" + str(int(time.time()))

    try:
        while 1:
            if (testing):
                # Use saved pointcloud file
                time_start = time.time()
                filename = "1.ply"
                pcd = o3d.io.read_point_cloud(filename)
                npCloud = np.asarray(pcd.points)
                # Flips points to align with those from the landrov
                offset = np.array([0.5, 0, -0.5])
                pcd = pc.npToPcd(npCloud * np.array([1, -1, -1]) + offset)
                # Simulate delays in recieving pointcloud
                time.sleep(0.1)
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
                print("_" * 60)
                # Downsample pointcloud
                downpcd = pcd.voxel_down_sample(voxel_size=0.01)

                stop = pc.PointCloud(downpcd, stopColour, stopMin, stopMax)
                center = pc.PointCloud(downpcd, centerColour, centerMin, centerMax)
                left_objects = pc.PointCloud(downpcd, leftColour, leftMin, leftMax)
                right_objects = pc.PointCloud(downpcd, rightColour, rightMin, rightMax)

                if (vehicle.following_side == "right"):
                    objects_to_follow = right_objects
                else:
                    objects_to_follow = left_objects

                vehicle.calculate_travel_time()
                distance_from_wall = None

                # Determine action for vehicle to take
                if vehicle.is_collision_with(stop) or vehicle.is_stuck(): 
                    # P0 and P1
                    command = "stop"
                elif vehicle.is_collision_with(center):
                    # P2 collision avoidance
                    command = veh.collision_avoidance_command(left_objects, right_objects)
                elif vehicle.is_collision_with(objects_to_follow):
                    # P3 plane estimation
                    plane_model, plane_pcd = pe.estimate_plane_equation(objects_to_follow)
                    distance_from_wall = pe.estimate_distance_from_wall(vehicle, plane_model)
                    angle_from_wall = pe.estimate_angle_to_turn(plane_model)
                    print("Distance from wall:", round(distance_from_wall, 2), "Target:", target_distance)
                    print("Angle to turn:", round(angle_from_wall, 2))

                    command = veh.distance_to_command(vehicle, distance_from_wall, angle_from_wall)
                    print("Plane Estimation Command:", command)

                    if command == "turn":
                        vehicle.calculate_travel_time(angle_from_wall)
                        if angle_from_wall < 0:
                            command = "left"
                        else:
                            command = "right"
                        print("Custom turn time:", round(vehicle.travel_time, 2))

                else:
                    # P3 Turn towards following side
                    command = vehicle.following_side

                # Print status
                print("Command:", command)
                template = "Points in stop zone {}, points left {}, points in center {}, points right {}"

                print(template.format(len(stop.pcd.points), len(left_objects.points), len(center.points), len(right_objects.points)))
                print("Time taken", round(time.time() - time_start, 2))

                updated_cloud = True

                # Display pointclouds
                if (visualising or save_clouds):
                    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
                    point_clouds = [mesh_frame, stop.pcd, center.pcd, left_objects.pcd, right_objects.pcd, 
                                    stop.bounding, left_objects.bounding, center.bounding, right_objects.bounding]
                    if distance_from_wall != None:
                        wall_location = distance_from_wall
                        if vehicle.following_side == "left":
                            wall_location *= -1
                        point_clouds.append(pc.getSphere(0.01, [0, 0, 0], wall_location))

                if visualising:
                    visualise.visualise(point_clouds)
                
                if save_clouds:
                    point_clouds_to_save = [pcd, stop.pcd, center.pcd, left_objects.pcd, right_objects.pcd]
                    visualise.save_point_clouds(point_clouds_to_save, custom_folder_name)

            if updated_cloud and not testing:
                if command == "stop":
                    print("Landrov cannot find clear path...")
                    print("Stopping")
                    vehicle.stop()
                    break
                elif command == "forward":
                    vehicle.reset_turn_counts()
                    vehicle.forward()
                    print("Going Forward")
                elif command == "right":
                    print("Turn Right")
                    vehicle.turn("right")
                elif command == "left":
                    print("Turn Left")
                    vehicle.turn("left")
                else:
                    print("No Command")
                    vehicle.stop()
                
                found_cloud = False
                updated_cloud = False

    except KeyboardInterrupt:
        print("Force Close")
        if not testing:
            vehicle.stop()
            # vehicle.sensor_socket.close()

main()