import numpy as np
import open3d as o3d


class PointCloud:
    """
    A pointcloud object for handling functions related to a pointcloud.
    """
    def __init__(self, pcd, colour, min_point=[0, 0, 0], max_point=[0, 0, 0]):
        self.min_point = min_point
        self.max_point = max_point
        self.pcd = self.filterPoints(pcd)
        self.points = self.pcd.points
        self.colour = colour
        self.bounding = []

        self.setBoundingRegion()
        self.colourPointCloud()
    
    def filterPoints(self, pcd):
        """
        Filters points in a cloud to be within a bounding box. 

        Args:
            pcd: The pointcloud to confine.

        Returns:
            Pointcloud: Pointcloud fit into bounding box. 
        """

        minX, minY, minZ = self.min_point
        maxX, maxY, maxZ = self.max_point
        npCloud = np.asarray(pcd.points)

        result = npCloud[np.where((npCloud[:,0] < maxX) & (npCloud[:,0] > minX) 
                                & (npCloud[:,1] < maxY) & (npCloud[:,1] > minY)
                                & (npCloud[:,2] < maxZ) & (npCloud[:,2] > minZ))[0]]

        return npToPcd(result)
    
    def colourPointCloud(self):
        self.pcd.paint_uniform_color(self.colour)

    def setBoundingRegion(self):
        """
        Creates the bounding region lines and colours. them.
        """

        sx, sy, sz = self.min_point
        bx, by, bz = self.max_point

        point1 = [sx, sy, sz]
        point2 = [bx, by, bz]
        point3 = [bx, sy, sz]
        point4 = [bx, by, sz]
        point5 = [sx, by, sz]
        point6 = [sx, by, bz]
        point7 = [sx, sy, bz]
        point8 = [bx, sy, bz]

        points = [point1, point2, point3, point4, point5, point6, point7, point8]

        lines = [[0, 2], [0, 6], [0, 4], [2, 3], [2, 7], [6, 7], 
                [6, 5], [4, 5], [4, 3], [3, 1], [1, 7], [5, 1]]

        colors = [self.colour for i in range(len(lines))]
        bounding_region = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        bounding_region.colors = o3d.utility.Vector3dVector(colors)
        self.bounding = bounding_region


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


def getSphere(radius, colour, distance):
    """
    Creates a pointcloud sphere for displaying in the Open3D visualiser.

    Args:
        radius: Radius of the sphere.
        colour: Colour of the sphere.
        distance: Distance of the sphere from the origin.

    Returns:
        Pointcloud: A mesh sphere object pointcloud.
    """

    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh_sphere.translate([distance, 0, 0])
    mesh_sphere.compute_vertex_normals()
    mesh_sphere.paint_uniform_color(colour)

    return mesh_sphere