import open3d as o3d
import os
import time


def view_pointclouds(pointclouds, vis, view_unprocessed_pcd):
    """
    A method to view pointclouds with the Open3D visualiser.

    Args:
        pointclouds: A list of pointcloud objects to visualise
        vis: An Open3D visualiser object.
        view_unprocessed_pcd: The downsampled pointcloud object
    """

    try:
        if view_unprocessed_pcd:
            vis.update_geometry(pointclouds[0])
        else:
            for i in range(1, len(pointclouds)):
                print("Points for", i, ":", len(pointclouds[i].points))
                vis.update_geometry(pointclouds[i])

        vis.poll_events()
        vis.update_renderer()
    except:
        print("WARNING: Issue visualising pointclouds")


def save_point_clouds(point_clouds, custom_folder_name=""):
    """
    Saves a pointcloud object to a .pcd file.

    Args:
        point_clouds: The pointcloud object to save.
        custom_folder_name: The foldername to save pointclouds to. Defaults to "".
    """

    time_stamp = time.time()
    folder_name = str(int(time_stamp * 100)) + "/"
    point_cloud_folder = "./pointclouds/"
    path = os.getcwd()

    try:
        os.mkdir(path + point_cloud_folder)
    except:
        pass

    if custom_folder_name != "":
        point_cloud_folder += custom_folder_name + "/"
        try:
            os.mkdir(path + point_cloud_folder)
        except:
            pass
    try:
        os.mkdir(path + point_cloud_folder + folder_name)
    except:
        print("ERROR: Issue creating timestamped folder")
    try:
        for i in range(0, len(point_clouds)):
            if len(point_clouds[i].points) <= 0:
                point_clouds[i].points.append([0, 0, 0])
            o3d.io.write_point_cloud(point_cloud_folder + folder_name + str(i) + ".pcd", point_clouds[i])
    except:
        print("WARNING: Issue saving pointclouds")


def get_capture_frame_folder_paths(capture_folder_path):
    """
    Takes a capture folder path name and gets a list of all pointcloud folders inside. 

    Args:
        capture_folder_path: String path to a capture folder path.

    Returns:
        List: A list of pointcloud folder filenames.
    """

    capture_frame_folder_paths = []

    for subdir, dirs, files in os.walk(capture_folder_path):
        for dir in dirs:
            capture_frame_folder_paths.append(os.path.join(subdir, dir))

    return capture_frame_folder_paths


def get_frame_pointclouds(frame_folder_path, folder_length=5):
    """
    Gets all pointclouds for a given frame in a saved capture.

    Args:
        frame_folder_path: Path to the capture frame folder.
        folder_length: How many pointcloud files are in the frame folder. Defaults to 5.

    Returns:
        List: List of pointclouds for a given frame.
    """

    pointclouds = []
    for i in range(0, folder_length):
        pcd = o3d.io.read_point_cloud(frame_folder_path + "/" + str(i) + ".pcd")
        pointclouds.append(pcd)
    
    return pointclouds


def visualise(point_clouds):
    """
    Visualises a list of pointclouds using the Open3D visualiser.

    Args:
        point_clouds: A list of pointclouds to visualise.
    """

    try:
        o3d.visualization.draw_geometries(point_clouds)
    except:
        print("WARNING: Issue visualising pointclouds")


def main():
    """
        Visualises a navigation capture session.
    """
    
    view_unprocessed_pcd = False
    frame_delay = 0.2
    pointcloud_folder_name = "pointclouds"
    capture_folder_name = "erskine_left_1.0_0.3_1598757834"
    capture_folder_path = "./" + pointcloud_folder_name + "/" + capture_folder_name + "/"

    capture_frame_folder_names = get_capture_frame_folder_paths(capture_folder_path)

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    vis.add_geometry(mesh_frame)
    point_clouds = []

    # Visualisation colours
    stopColour = [1, 0, 0]
    centerColour = [0, 0, 1]
    leftColour = [0, 1, 0]
    rightColour = [0, 0, 0]

    ctr = vis.get_view_control()
    parameters = o3d.io.read_pinhole_camera_parameters("camera.json")
    
    first = True

    for i in range(0, len(capture_frame_folder_names)):
        frame_pointclouds = get_frame_pointclouds(capture_frame_folder_names[i])

        if first:
            point_clouds = frame_pointclouds
            if view_unprocessed_pcd:
                 vis.add_geometry(point_clouds[0])
            else:
                for i in range(1, len(frame_pointclouds)):
                    vis.add_geometry(frame_pointclouds[i])
            ctr.convert_from_pinhole_camera_parameters(parameters)
            first = False
        else:        
            for i in range(0, len(frame_pointclouds)):
                point_clouds[i].points = frame_pointclouds[i].points

        point_clouds[1].paint_uniform_color(stopColour)
        point_clouds[2].paint_uniform_color(centerColour)
        point_clouds[3].paint_uniform_color(leftColour)
        point_clouds[4].paint_uniform_color(rightColour)

        view_pointclouds(point_clouds, vis, view_unprocessed_pcd)
        time.sleep(frame_delay)

    vis.destroy_window()

# main()