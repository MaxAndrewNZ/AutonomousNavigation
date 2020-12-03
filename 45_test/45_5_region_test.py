import sys
sys.path.insert(1, '../side_camera_test/')
import time,zmq,pickle
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
import vehicle as veh
import cv2
import math
from matplotlib import pyplot as plt
import random
import os

class Region:
    def __init__(self, min_x, max_x, distance_limits):
        self.min_x = min_x
        self.max_x = max_x
        self.close_dist, self.med_dist, self.inf_dist = distance_limits

        self.current_dist = "inf"
        self.closest_point = math.inf

    def set_depth(self, depth_chunk):
        #TODO: Allow for a little noise.
        depth_area = depth_chunk[self.min_x: self.max_x]
        closest_point = min(depth_area)
        self.closest_point = closest_point # will have [x, z]
        self.update_dist_reading()    

    def update_dist_reading(self):
        if self.closest_point <= self.close_dist:
            self.current_dist = "close"
        elif self.closest_point <= self.med_dist:
            self.current_dist = "med"
        else:
            self.current_dist = "inf"

    def get_closest_point(self):
        return self.closest_point

def get_minimal_distance(regions):
    minimal_distance = math.inf
    closest_region_index = 0
    for i in range(0, len(regions)):
        current_distance = regions[i].get_closest_point()[1]
        if current_distance < minimal_distance:
            minimal_distance = current_distance
            closest_region_index = i

    return minimal_distance

def evaluate_distances(regions, depth, region_size):
    #TODO: Flatten the depth image and cut into wedges
    flattened_depth = depth
    for i in range(0, len(regions)):
#        depth_chunk = flattened_depth[region_size*i, region_size*(i+1)]
        regions[i].set_depth(flattened_depth)

    plt.imshow(depth,'gray')
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
        vel_full

    return velocity

def main():
    """   
    This method is the heart of the Landrov navigation. 
    It handles controling the multi-step navigation. 
    The navigation settings can be configured within this.
    """
  
    ############      Configuration        ##################
    testing = False
    visualising = False
    save_clouds = False
    custom_folder_name = "testing_45"

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
    main_colour = [1, 0, 0]
    lag_colour = [0, 1, 0]
    lead_colour = [0, 0, 1]

    # Regions
    num_regions = 5
    angle_range = 180
    x_max = 365
    region_size = x_max / 5

    close_dist = 0.5
    med_dist = 1.0
    inf_dist = 2.0
    distance_limits = [close_dist, med_dist, inf_dist]

    back_left_region = Region(0, region_size, distance_limits)
    left_region = Region(region_size, region_size * 2, distance_limits)
    front_left_region = Region(region_size * 2, region_size * 3, distance_limits)
    front_region = Region(region_size * 3, region_size * 4, distance_limits)
    front_right_region = Region(region_size * 4, region_size * 5, distance_limits)

    regions = [back_left_region, left_region, front_left_region, front_region, front_right_region]

    # Proportional and derivative constants
    p_const = 15
    d_const = 0

    linear_errors = []
    angular_errors = []

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

    try:
        while 1:
            if (testing):
                #TODO: Set saved depth data.
                time_start = time.time()
                depth = None
                time.sleep(0.1)
                found_cloud = True
       
            if not found_cloud:
                vehicle.connect_depth_raw()
                while not found_cloud:
                    if len(zmq.select([vehicle.sensor_socket],[],[],0)[0]):
                        topic,buf = vehicle.sensor_socket.recv_multipart()
                        if topic == b'depthraw':
                            time_start = time.time()
                            depth = cv2.imdecode(buf, dtype=np.float32)
                            found_cloud = True

                vehicle.sensor_socket.close()

            else:
                print("Depth:", depth)
                updated_cloud = True
                print("Dept size:", depth.size())

                x_max = depth.size()[0]

                if (visualising):
                    #TODO: 2D visualisation
                    print("No visualisation implemented")

                #TODO: Implement wedge navigation
                evaluate_distances(regions, depth, region_size)
                min_distance, min_distance_index = get_minimal_distance(regions)
                current_time = time.time()
                min_dist_x_value = regions[min_distance_index].closest_point[1]
                min_dist_angle = (min_dist_x_value / x_max) * 2 * math.pi
                linear_velocity = set_velocity(min_distance, min_dist_angle, target_distance, max_speed)

                # Wall distance error
                d_error = min_distance - target_distance
                if len(linear_errors > 0):
                    linear_errors.append((0, current_time() - 100))
                linear_pd = (p_const * d_error) + d_const * ((d_error - linear_errors[-1][0]) / (current_time - linear_errors[-1][1]))
                linear_errors.append((d_error, current_time))

                # Wall angle error
                a_error = min_dist_angle - (math.pi / 2)
                angular_p = p_const * a_error
                angular_errors.append((a_error, current_time))

                # Calculate angular velocity
                angular_velocity = max(min(linear_pd + angular_p, angular_limit), - angular_limit)

                print("Linear velocity:", linear_velocity)
                print("Angular velocity:", angular_velocity)

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

        plt.scatter(*zip(*linear_errors))
        plt.show()
        plt.scatter(*zip(*angular_errors))
        plt.show()

main()