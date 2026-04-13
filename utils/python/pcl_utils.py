import numpy as np
import cv2 as cv
import glob
import json
import os
import open3d as o3d
from scipy.spatial.transform import Rotation

# -----------------------------------------------Point Clouds functions----------------------------------------------- #

def get_LiDAR_data_from_file(file_path, file_type='.pcd'):
    """
    Load data from a file.

    Inputs:
    - file_path: str
        The path to the file containing the data (X, Y, Z, Intensity).

    Returns:
    - points: np.ndarray
        The loaded points without the intensity.
    - intensity: np.ndarray
        The loaded intensity values corresponding to the points.
    """
    # Get name of the file
    file_name = os.path.basename(file_path)
    if file_type == '.pcd':
        try : 
            # Lire le fichier PCD avec le module tensor
            pcd = o3d.t.io.read_point_cloud(file_path)
            
            # Extraire les points et l'intensité
            points = np.asarray(pcd.point.positions.cpu().numpy())  # [x, y, z]
            if "intensity" in pcd.point:
                intensity = np.asarray(pcd.point.intensity.cpu().numpy())  # Extraire l'intensité
            else:
                raise ValueError("Le fichier PCD ne contient pas d'intensité.")
        except Exception as e :
            print(f"Error when loading file :{e}. File seems empty.")
            points, intensity = None, None
    elif file_type == '.txt':
        # Lire le fichier txt
        read_data = np.loadtxt(file_path, delimiter=',', skiprows=1)[:, :4]  # [x, y, z, intensity]
        # Extraire les points et l'intensité        
        points = read_data[:, :3]  # [x, y, z]      
        intensity = read_data[:, 3]  # [intensity]            
    
    # Combiner les données dans un tableau numpy [x, y, z, intensity]
    # data = np.hstack((points, intensity.reshape(-1, 1)))  # [x, y, z, intensity]
    return points, intensity, file_name


def get_corresponding_pc_from_image(image:str, pointclouds:list):
    """
    Find the closest pointclouds to an image comparing names of the files which corresponds to timestamps.
    """
    mini = int(os.path.splitext(image)[0])
    idx_mini = 0
    for idx, pcl in enumerate(pointclouds):
        diff = abs(int(os.path.splitext(image)[0]) - int(os.path.splitext(pcl)[0]))
        if diff <= mini:
            mini = diff
            idx_mini = idx
    return idx_mini

def filter_point_cloud(pcd, x_range=(-1, 1), y_range=(-1, 1)):
    """
    Filter a point cloud to keep only points within the specified x and y ranges.

    Inputs:
    - pcd: o3d.geometry.PointCloud
        The point cloud to filter.
    - x_range: tuple (default: (-1, 1))
        The range for x coordinates (min, max).
    - y_range: tuple (default: (-1, 1))
        The range for y coordinates (min, max).

    Returns:
    - filtered_pcd: o3d.geometry.PointCloud
        The filtered point cloud.
    """
    # Convert the point cloud to a numpy array
    points = np.asarray(pcd.points)

    # Create a boolean mask for points within the specified x and y ranges
    x_min, x_max = x_range
    y_min, y_max = y_range
    mask = (
        (points[:, 0] >= x_min) &
        (points[:, 0] <= x_max) &
        (points[:, 1] >= y_min) &
        (points[:, 1] <= y_max)
    )

    # Apply the mask to the points
    filtered_points = points[mask]

    # Create a new point cloud with the filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    # If the original point cloud has colors, apply the same mask to the colors
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_colors = colors[mask]
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    return filtered_pcd


def display_points_in_image(image, intrinsic_mtx, points, intensity, T, count, dist_coef=np.array([0, 0, 0, 0, 0]), extension='.pcd'):
    """
    Display LiDAR points on an image.

    Parameters:
    - image: path to the image
        The image on which to display the points.
    - points: np.ndarray
        The LiDAR points to display.
    - intensity: np.ndarray
        The intensity values corresponding to the points.
    - T : np.ndarray
        The transformation between the LIDAR and the image
    - count : int
        The image count used for the saving
    """
    # print("Len Intensity : ", len(intensity))
    # print("Len points : ", len(points))
    # Load the image
    init_image = cv.imread(image)
    image = cv.imread(image)
    if image is None:
        print(f"Error: Could not load image {image}")
        return
    height, width = image.shape[:2]
    # Apply transformation to LiDAR points
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))  # Convert to homogeneous coordinates
    transformed_points_homogeneous = T @ points_homogeneous.T  # Apply transformation
    transformed_points = (transformed_points_homogeneous[:3, :] / transformed_points_homogeneous[3, :])[:3,:].T  # Convert back to Euclidean coordinates
    # print("Transformed homogeneous points:\n", transformed_points_homogeneous[0, :])
    projected_corners_camera = cv.projectPoints(transformed_points, np.zeros((3,)), np.zeros((3,)), intrinsic_mtx, dist_coef)[0].reshape(-1, 2)  # Project points to image plane
    for i in range(len(transformed_points)):
        points = projected_corners_camera[i]
        if 0 <= points[0] < width and 0 <= points[1] < height and not np.isnan(points[0]) and not np.isnan(points[1]):
            try :
                intensity_value = intensity[i]
                if extension=='.pcd':
                    intensity_value = int(intensity[0])
            except :
                # print(f"Insity value out of bounds, idx {i} on {intensity.size} size array. Setting intenisty to 0")
                intensity_value = 0
            # print("Intensity value : ", intensity_value)
            # print("Max Intensity value : ", np.max(intensity))
            # print("Min Intensity value : ", np.min(intensity))
            # Normalize intensity to [0, 255]
            # intensity_value = int((intensity_values - np.min(intensity)) / (np.max(intensity) - np.min(intensity)) * 255)
            # print("Point:", points, "Intensity:", intensity_value)
            cv.circle(image, (int(points[0]), int(points[1])), 1, (255, 0, intensity_value), -1)  # Draw point on image
    
    # cv.imshow("Original Image", init_image)
    cv.imshow(f"LiDAR Points on cam {count}", image)
    cv.waitKey(5)
    return image

# -----------------------------------------------Motion Capture functions----------------------------------------------- #

def get_MoCap_data_from_file(file_path, file_type='.txt'):
    """
    Load data from a file.

    Inputs:
    - file_path: str
        The path to the file containing the data (X, Y, Z, Intensity).

    Returns:
    - mocap_poses: map(int, np.ndarray)
        A dictionnary of the position given by the MoCap in the  MoCap frame.
    - mocap_orient: map(int, np.ndarray)
        A dictionnary of the quaternions given by the MoCap in the MoCap frame.
    """
    if file_type == '.txt':
        # Lire le fichier txt
        read_data = np.loadtxt(file_path, delimiter=',', skiprows=1)[:, :8]  # [id, x, y, z, qx, qy, qz, qw]
        ids = read_data[:, 0]
        poses = read_data[:, 1:4]
        quats = read_data[:, 4:]
        mocap_poses = {int(ids[i]): tuple(poses[i, :]) for i in range(len(ids))}
        mocap_orient = {int(ids[i]): tuple(quats[i, :]) for i in range(len(ids))}

    return mocap_orient, mocap_poses

def quaternion_to_rotation_matrix(quaternion):
    """
    Convert a quaternion into a 3x3 rotation matrix.

    Parameters:
        quaternion (np.array): [x, y, z, w] quaternion

    Returns:
        np.array: 3x3 rotation matrix
    """
    x, y, z, w = quaternion
    # Normalize the quaternion
    norm = np.linalg.norm(quaternion)
    if norm > 0:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

    # Compute the rotation matrix
    rotation_matrix = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    return rotation_matrix

def matrix_from_translation_and_quaternion(translation, quaternion):
    """Convert translation and quaternion to a 4x4 transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    T[:3, 3] = translation
    return T

def moCap_to_Transfo(position, quaternion):
    """
    Convert a position and quaternion into a 4x4 homogeneous transformation matrix.

    Parameters:
        position (np.array): [x, y, z] translation vector
        quaternion (np.array): [x, y, z, w] quaternion

    Returns:
        np.array: 4x4 homogeneous transformation matrix
    """
    quat = np.array(quaternion)
    pose = np.array(position)
    rotation_matrix = quaternion_to_rotation_matrix(quat)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = pose
    return transform_matrix

def get_corresponding_MoCap_from_pcd(mocaps:list, pointclouds:str):
    """
    Find the closest Mocap file to a pointcloud comparing names of the files which corresponds to timestamps.
    """
    mini = int(os.path.splitext(pointclouds)[0])
    idx_mini = 0
    for idx, mocap in enumerate(mocaps):
        diff = abs(int(os.path.splitext(pointclouds)[0]) - int(os.path.splitext(mocap)[0]))
        if diff <= mini:
            mini = diff
            idx_mini = idx
    return idx_mini