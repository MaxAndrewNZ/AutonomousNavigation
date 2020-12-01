import math


def estimate_plane_equation(pointcloud):
    """
    Takes a pointcloud and places a plane of best fit to it.
    The plane model and cloud is returned.

    Args:
        pointcloud: A pointclould to estimate a plane within.

    Returns:
        Plane Model: A list of parameters to the planes model.
        Pointcloud: A pointcloud of all points that lays within the plane.
    """

    plane_model, inliers = pointcloud.pcd.segment_plane(distance_threshold=0.1,
                                            ransac_n=3,
                                            num_iterations=1000)

    # This method has a different name depending on the o3d version
    try:
        inlier_cloud = pointcloud.pcd.select_by_index(inliers)
    except:
        inlier_cloud = pointcloud.pcd.select_down_sample(inliers) 

    inlier_cloud.paint_uniform_color([0, 1, 1])

    return plane_model, inlier_cloud


def estimate_distance_from_wall(vehicle, plane_model):
    """
    Estimates the vehicles distance from a plane.

    Args:
        vehicle: The vehicle object.
        plane_model: List of the estimated planes parameters.

    Returns:
        Float: Estimated distance from a wall in metres.
    """

    [a, b, c, d] = plane_model
    distance_from_wall = - d / a

    if vehicle.following_side == "left":
        distance_from_wall *= -1
                        
    return distance_from_wall


def estimate_angle_to_turn(plane_model):
    """
    Estimates the vehicles angle relative to a plane.

    Args:
        plane_model: List of the estimated planes parameters.

    Returns:
        Float: Estimated angle of vehicle from plane.
    """

    [a, b, c, d] = plane_model
    angle_to_turn = math.degrees(math.atan((a / - c)))

    if angle_to_turn > 0:
        angle_to_turn -= 90
    else:
        angle_to_turn += 90

    return - angle_to_turn