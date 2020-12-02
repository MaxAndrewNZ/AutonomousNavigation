import time,zmq,pickle
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
import pointcloud as pc
import vehicle as veh
import visualise_pointclouds as visualise
import plane_estimation as pe
import math
from ast import literal_eval
import random
import os


def calculate_wall_angle(left_z, right_z):
    #TODO: Check this angle calculation is correct.
    delta_x = 2
    delta_y = left_z - right_z

    return math.degrees(math.atan2(delta_y, delta_x))


def main():
    """   
    This method is the heart of the Landrov navigation. 
    It handles controling the multi-step navigation. 
    The navigation settings can be configured within this.
    """
  
    ############      Configuration        ##################
    testing = True
    visualising = True
    save_clouds = False
    custom_folder_name = "testing_side"

    following_side = "left" # left or right
    target_distance = 0.4 # Meters
    error_distance = 0.1 # Meters
    error_angle = 10.0 # Degrees

    speed = 0.3 # Speed of the vehicle
    turn_speed = 0.5 # Speed * 2 normally
    turn_time_multiplier = 0.0 # Consecutive turns get bigger
    maximum_turns = 10 # Vehicle will stop after turning this many times in a row
    min_points_for_avoidance = 80 # Increase if navigation is disrupted due to noise

    # Pointcloud region configuration
    # X
    centerWidth = 5
    rightMinX = centerWidth / 2
    leftMaxX = -rightMinX

    # Z
    stopMinZ = 0.0
    stopMaxZ = 1.0

    # Y
    minY = -0.6
    maxY = 0.0

    # Visualisation region colours37
    main_colour = [1, 0, 0]
    lag_colour = [0, 1, 0]
    lead_colour = [0, 0, 1]

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

    stopMin = [leftMaxX, minY, stopMinZ]
    stopMax = [rightMinX, maxY, stopMaxZ]

    leftMin = [-2, minY, stopMinZ]
    leftMax = [0, maxY, stopMaxZ]

    rightMin = [0, minY, stopMinZ]
    rightMax = [2, maxY, stopMaxZ]

    custom_folder_name += "_" + following_side + "_" + str(target_distance) + "_" + str(speed) + "_" + str(int(time.time()))
    time_start = time.time()

    try:
        while 1:
            if (testing):
                # Use saved pointcloud file
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
                downpcd = pcd.voxel_down_sample(voxel_size=0.01)

                cloud = pc.PointCloud(downpcd, main_colour, stopMin, stopMax)
                lag = pc.PointCloud(downpcd, lag_colour, leftMin, leftMax)
                lead = pc.PointCloud(downpcd, lead_colour, rightMin, rightMax)

                # Print status
                template = "Points in cloud {}, x1 = {}, x2 = {}"

                print(template.format(len(cloud.pcd.points), len(lag.pcd.points), len(lead.pcd.points)))
                print("Time taken", round(time.time() - time_start, 2))
                time_start = time.time()

                updated_cloud = True

                # Display pointclouds
                if (visualising or save_clouds):
                    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
                    point_clouds = [mesh_frame, lag.pcd, lead.pcd, lag.bounding, lead.bounding]

                if visualising:
                    visualise.visualise(point_clouds)

                lag_points = np.asarray(lag.pcd.points)
                lead_points = np.asarray(lead.pcd.points)

                min_avoid = vehicle.min_points_for_avoidance

                if lag_points < min_avoid and lead_points < min_avoid:
                    # S0: Cant find any wall
                    print("No wall found")
                    command = "stop"
                elif lag_points > min_avoid and lead_points < min_avoid:
                    #S1: End of row. Therefore complete end of row turn.
                    print("End of row")
                    #TODO: End of row turn.
                    vehicle.calculate_travel_time(180)
                    command = "left"
                elif lead_points > min_avoid:
                    # S2 and S3: Following row or starting to follow row.
                    print("Follow wall")
                    # TODO: plane estimation and correction on leading region
                    plane_model, _ = pe.estimate_plane_equation(lead)
                    distance = pe.estimate_distance_from_wall(plane_model)
                    angle = pe.estimate_angle_to_turn(plane_model)
                    if distance > target_distance + error_distance:
                        # Not at the target distance from the wall. Turn left to fix this.
                        # TODO: Make this turn less aggressive and random
                        print("Too far")
                        command = "left"
                        vehicle.travel_time = 0.3
                    elif distance < target_distance - error_distance:
                        # Not at the target distance from the wall. Turn right to fix this.
                        print("Too close")
                        command = "right"
                        vehicle.travel_time = 0.3
                    else:
                        if angle > error_angle:
                            print("Turning to follow wall.")
                            vehicle.calculate_travel_time(angle)
                            if angle > 0:
                                command = "left"
                            else:
                                command = "right"
                        else:
                            command = "forward"
                else:
                    command = "stop"

                print("Command:", command)

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

main()