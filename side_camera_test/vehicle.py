import numpy as np
import time,zmq,pickle
import math


class Vehicle:
    """
    Class for handing vehicle control and variables.
    """
    def __init__(self, ip, control_port, sensor_port):
        self.ip = ip
        self.control_port = control_port
        self.sensor_port = sensor_port
        self.context = zmq.Context()
        self.control_socket = None
        self.sensor_socket = None

        self.speed = 0.0
        self.following_side = "right"
        self.target_distance = 1.0
        self.error_distance = 0.2
        self.error_angle = 10.0
        self.min_points_for_avoidance = 100

        self.left_turn_count = 0
        self.right_turn_count = 0
        self.turn_speed = 0.0
        self.turn_time_multiplier = 0.0
        self.maximum_turns = 0
        self.travel_time = 0.0
        self.wheel_distance_between = 0.35
        self.velocity_multiplier = 0.7
        self.max_range = 3.0
    
    def set_control_variables(self, speed, turn_speed, turn_time_multiplier, maximum_turns):
        self.speed = speed
        self.turn_speed = turn_speed
        self.turn_time_multiplier = turn_time_multiplier
        self.maximum_turns = maximum_turns

    def set_following_variables(self, following_side, target_distance, error_distance, error_angle):
        self.following_side = following_side
        self.target_distance = target_distance
        self.error_distance = error_distance
        self.error_angle = error_angle

    def connect_control(self):
        self.control_socket = self.context.socket(zmq.PUB)
        self.control_socket.connect("tcp://192.168.8.106:5556")
        # self.control_socket.connect(self.ip + ":" + self.control_port)
    
    def connect_pointcloud(self):
        self.sensor_socket = self.context.socket(zmq.SUB)
        self.sensor_socket.connect("tcp://192.168.8.106:5557")
        # self.sensor_socket.connect(self.ip + ":" + self.sensor_port)
        self.sensor_socket.setsockopt(zmq.SUBSCRIBE,b'pointcloud')
    
    def set_min_points_for_avoidance(self, min_points_for_avoidance):
        self.min_points_for_avoidance = min_points_for_avoidance

    def calculate_travel_time(self, angle=0):
        """
        Caluclates the time to turn depending on a provided angle.

        Args:
            angle: The angle to turn. Defaults to 0.
        """
        max_turn_time = 3
        if angle == 0:
            turns = self.left_turn_count + self.right_turn_count
            self.travel_time = self.turn_speed + self.turn_time_multiplier * turns
        else:
            velocity = self.turn_speed * self.velocity_multiplier
            print("Velocity:", velocity)
            wheel_distance_circumference = self.wheel_distance_between * math.pi
            distance = wheel_distance_circumference * (abs(angle) / 360)

            if velocity <= 0:
                self.travel_time = 0
            else:
                self.travel_time = min(max_turn_time, distance / velocity)

    def is_stuck(self):
        """
        If the vehicle is stuck. It will stop.

        Returns:
            Boolean: True if stuck, False otherwise.
        """
        if self.right_turn_count > self.maximum_turns and self.left_turn_count > self.maximum_turns:
            print("Cannot find clear path...")
            return True
        else:
            return False

    def is_collision_with(self, stop):
        return len(stop.points) > self.min_points_for_avoidance

    def reset_turn_counts(self):
        self.left_turn_count = 0
        self.right_turn_count = 0

    def forward(self):
        cmd = (self.speed, self.speed)
        self.control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)])

    def stop(self):
        cmd = (0.0, 0.0)
        self.control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)])
        self.control_socket.close()

    def turn(self, direction):
        """
        Turns the vehicle in the set following direction.

        Args:
            direction: string of following side, left or right.
        """
        old_cmd = (self.speed, self.speed)
        if direction == "left":
            self.left_turn_count += 1
            cmd = (-self.turn_speed, self.turn_speed)
        else:
            cmd = (self.turn_speed, -self.turn_speed)
            self.right_turn_count += 1

        self.control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)])
        start_time = time.time()
        time_spent = 0
        while (time_spent < self.travel_time):
            time_spent = time.time() - start_time

        self.control_socket.send_multipart([b'motor',pickle.dumps(old_cmd,0)])


def distance_to_command(veh, distance, angle):
    """
    Takes a vehicle, distance and angle and determines the next command.

    Args:
        veh: The Vehicle object.
        distance: Distance from the wall.
        angle: Angle from the wall.

    Returns:
        command: String of the vehicles next command. For example, turn.
    """

    if veh.following_side == "left":
        non_following_side = "right"
    else:
        non_following_side = "left"

    if abs(distance) > veh.max_range:
        print("Unlikely plane estimation...")
        command = "forward"
    elif distance > veh.target_distance + veh.error_distance:
        print("Fixing distance...")
        command = veh.following_side
    elif distance < veh.target_distance - veh.error_distance:
        print("Fixing distance...")
        command = non_following_side
    elif angle > veh.error_angle or angle < -veh.error_angle:
        print("Fixing angle...")
        command = "turn"
    else:
        command = "forward"

    return command


def collision_avoidance_command(left_objects, right_objects):
    """
    Determines what way the vehicle should turn.
    This is done depending on the left and right pointclouds.

    Args:
        left_objects: Pointcloud of the left navigation region.
        right_objects: Pointcloud of the right navigation region.

    Returns:
        Command: A String command for the vehicles next navigation move.
    """

    if len(left_objects.points) > len(right_objects.points):
        command = "right"
    else:
        command = "left"

    return command


