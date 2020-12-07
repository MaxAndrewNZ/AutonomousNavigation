from matplotlib import pyplot as plt
import matplotlib.lines as mlines
from matplotlib import cm
import numpy as np

def display_line_follow(flat_cloud, a, b, region_min, region_max, linear_errors, angular_errors, linear_velocities, angular_velocities):
    plt.ion()
    plt.show()
    plt.clf()

    plt.subplot(3, 1, 1)
    plt.scatter(flat_cloud[:,0], flat_cloud[:,1], color='grey')
    plt.plot([0, 0], [0, region_max[2]], color='green')
    
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = a + b * x_vals
    plt.plot(x_vals, y_vals, color='red')

    plt.xlim(region_min[0], region_max[0])
    plt.ylim(region_min[2], region_max[2])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X distance')
    plt.ylabel('Z distance')
    plt.title('Top down view')

    plt.subplot(3, 2, 3)
    plt.plot(*zip(*linear_errors), color='blue')
    plt.ylim(-3, 3)
    plt.ylabel('Linear Error')

    plt.subplot(3, 2, 4)
    plt.plot(*zip(*linear_velocities), color='black')
    plt.ylim(-1, 1)
    plt.ylabel('Linear Velocity')

    plt.subplot(3, 2, 5)
    plt.plot(*zip(*angular_errors), color='blue')
    plt.ylim(-1.57, 1.57)
    plt.xlabel('time (seconds)')
    plt.ylabel('Angular Error')

    plt.subplot(3, 2, 6)
    plt.plot(*zip(*angular_velocities), color='black')
    plt.ylim(-2, 2)
    plt.ylabel('Angular Velocities')

    plt.draw()
    plt.pause(0.001)

def display_data(pointcloud, closest_point, region_min, region_max, linear_errors, angular_errors, linear_velocities, angular_velocities):
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

    plt.subplot(3, 2, 3)
    plt.plot(*zip(*linear_errors), color='blue')
    plt.ylim(-3, 3)
    plt.ylabel('Linear Error')

    plt.subplot(3, 2, 4)
    plt.plot(*zip(*linear_velocities), color='black')
    plt.ylim(-1, 1)
    plt.ylabel('Linear Velocity')

    plt.subplot(3, 2, 5)
    plt.plot(*zip(*angular_errors), color='blue')
    plt.ylim(-1.57, 1.57)
    plt.xlabel('time (seconds)')
    plt.ylabel('Angular Error')

    plt.subplot(3, 2, 6)
    plt.plot(*zip(*angular_velocities), color='black')
    plt.ylim(-2, 2)
    plt.ylabel('Angular Velocities')

    plt.draw()
    plt.pause(0.001)

def display_errors(linear_errors, angular_errors):
    plt.subplot(2, 1, 1)
    plt.plot(*zip(*linear_errors))
    plt.ylim(-3, 3)
    plt.ylabel('Linear Error')
    plt.title('Linear and angular error over time')

    plt.subplot(2, 1, 2)
    plt.plot(*zip(*angular_errors))
    plt.ylim(-1.57, 1.57)
    plt.xlabel('time (seconds)')
    plt.ylabel('Angular Error')

    plt.show()