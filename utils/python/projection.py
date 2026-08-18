import os
import numpy as np
import json
import cv2 as cv
from image_utils import *
from pcl_utils import *

# Get parameters from the chessboard calibration
def readT_cam_Lidar(json_file:str):
    with open(json_file, "r") as f:
        data = json.load(f)
    T_lidar_image = np.array(data["transformation_matrix"])
    return T_lidar_image

def readT_cam_calib(json_file:str):
    with open(json_file, "r") as f:
        data = json.load(f)
    K = np.array(data["intrinsic_matrix"])
    dist_coeff = np.array(data["distortion_coefficients"])
    return K, dist_coeff

def get_T_Lid_cam_Mocap(T_Mo_Cam, T_Mo_Lid):
    T_Lid_cam = np.linalg.inv(T_Mo_Cam)@T_Mo_Lid
    return T_Lid_cam

def project_launch_dataset(path_to_dataset:str, sequence:str):
    """
    Description:
    The `project_launch_dataset` function projects LiDAR point clouds onto synchronized camera images. It aligns LiDAR data with 
    camera frames, and projects the points into each image using the MoCap positions.

    Inputs:
    - `png_rotated` (bool): Whether the images chosen are the .png ones, already rotated in the same orientation.

    Outputs:
    - Displays concatenated images with projected point clouds.
    """
    sequence_file = sequence
    LiDARs_path = f'{path_to_dataset}/{sequence_file}/lidars/full_pointclouds/full_extracted/'
    json_file = f"{path_to_dataset}/{sequence_file}/camera_parameters.json"
    file_encoding = '.pcd'
    cam_idxx = [201, 202, 203, 204, 205]
    images_choosen = []
    for idx in cam_idxx:
        LiDAR_pcl_names = []
        LiDAR_data = []
        cam_idx = idx
        images_path = f'{path_to_dataset}/{sequence_file}/png_images/cam_{cam_idx}/'
        images = sorted(os.listdir(images_path))
        for file_name in sorted(os.listdir(LiDARs_path)):
            if file_name.endswith(file_encoding):
                LiDAR_pcl_names.append(file_name)
                file_path = os.path.join(LiDARs_path, file_name)
                points, intensity, file_name = get_LiDAR_data_from_file(file_path, file_encoding)
                LiDAR_data.append([points, intensity])

        print("Size of LiDAR data:", len(LiDAR_data))
        print("Size of images:", len(images))
        T, K, res, dist_coef = readParams(json_file, cam_idx) # Extrinsic and Intrinsinc matrixes

        T_proj = np.linalg.inv(T) # Points_{cam} = T_cam{ref}⁻¹@ P_{ref}

        for i in range(len(images)):
            idx_lidar = get_corresponding_pc_from_image(images[i], LiDAR_pcl_names)
            print("Lidar choosen : ", LiDAR_pcl_names[idx_lidar])
            if type(LiDAR_data[idx_lidar][0]) == np.ndarray and type(LiDAR_data[idx_lidar][1])==np.ndarray:
                print(f"Index :{i+1}/{len(images)}")
                img_pc = display_points_in_image(os.path.join(images_path, images[i]), K, LiDAR_data[idx_lidar][0], LiDAR_data[idx_lidar][1], T_proj, cam_idx, dist_coef)
                if i == 200:
                    images_choosen.append(img_pc)
        cv.destroyAllWindows()
    print("==========================DISPLAY 5 IMAGES WITH POINTCLOUDS==========================")
    target_height = 300  # Set your desired height
    resized_images = []
    for k, img in enumerate(images_choosen):
        h, w = img.shape[:2]
        scale = target_height / h
        resized_img = cv.resize(img, (int(w * scale), target_height))
        resized_images.append(resized_img)
    # Concatenate images horizontally
    concatenated_image = np.hstack(resized_images)

    # Display the concatenated image
    cv.imshow("All Images", concatenated_image)
    cv.waitKey(0)  # Press any key to close the window
    cv.destroyAllWindows()


def project_calibration_dataset(path_to_dataset:str, cam_idx:int):
    """
    Description:

    The `project_launch_dataset` function projects LiDAR point clouds onto synchronized camera images. It aligns LiDAR data with 
    camera frames, and projects the points into each image using the chessboard calibration and the MoCap data to compare the projections.

    Inputs:
    - cam_idx : Wanted camera (202, 204, 205)
    - path_to_dataset : Root direction of the dataset

    Outputs:
    - Displays concatenated images with projected point clouds.
    """
    map_lidar_cam = {202:108, 204:178, 205:152}
    calib_folder = f"calib_mocap_lidar_{map_lidar_cam[cam_idx]}_cam_id_{cam_idx}"
    LiDARs_path = f'{path_to_dataset}/calibration/{calib_folder}/pointclouds/lidar_192_168_2_{map_lidar_cam[cam_idx]}'
    MoCap_path = f'{path_to_dataset}/calibration/{calib_folder}/pointclouds/mocap_poses/'
    json_file_cam_calib = f"{path_to_dataset}/calibration/{calib_folder}/results_chessboard_calib/camera_calib.json"
    json_file_T_cam_Lidar = f"{path_to_dataset}/calibration/{calib_folder}/results_chessboard_calib/T_Camera_Lidar_weighted.json"
    print(LiDARs_path)
    file_encoding = '.txt'
    LiDAR_pcl_names = []
    LiDAR_data = []
    images_path = f"{path_to_dataset}/calibration/{calib_folder}/cam_{cam_idx}/"
    images = sorted(os.listdir(images_path))
    for file_name in sorted(os.listdir(LiDARs_path)):
        if file_name.endswith(file_encoding):
            LiDAR_pcl_names.append(file_name)
            file_path = os.path.join(LiDARs_path, file_name)
            points, intensity, file_name = get_LiDAR_data_from_file(file_path, file_encoding)
            LiDAR_data.append([points, intensity])

    print("Size of LiDAR data:", len(LiDAR_data))
    print("Size of images:", len(images))
    print("=========================PROJECTION USING CHESSBOARD CALIBRATION=========================")
    T = readT_cam_Lidar(json_file_T_cam_Lidar) # Extrinsic and Intrinsinc matrixes
    K, dist_coeff = readT_cam_calib(json_file_cam_calib)

    for i in range(len(images)):
        idx_lidar = get_corresponding_pc_from_image(images[i], LiDAR_pcl_names)
        print("Lidar choosen : ", LiDAR_pcl_names[idx_lidar])
        if type(LiDAR_data[idx_lidar][0]) == np.ndarray and type(LiDAR_data[idx_lidar][1])==np.ndarray:
            print(f"Index :{i}/{len(images)}")
            display_points_in_image(os.path.join(images_path, images[i]), K, LiDAR_data[idx_lidar][0], LiDAR_data[idx_lidar][1], T, cam_idx, dist_coeff, extension='.txt')
    cv.destroyAllWindows()

    print("=========================PROJECTION USING MOTION CAPTURE=========================")
    mocap_file = sorted(os.listdir(MoCap_path))[0]
    mocap_orient, mocap_poses = get_MoCap_data_from_file(os.path.join(MoCap_path, mocap_file))
    T_Mo_Lid = moCap_to_Transfo(mocap_poses[cam_idx], mocap_orient[cam_idx])
    t_offset_cam = np.array([[1.71162347745*0.001], [10.0*0.001], [-74.5*0.001]]) # Adding an offset manually
    T_Mo_cam = T_Mo_Lid.copy()
    T_Mo_cam[:3, 3]+=t_offset_cam.reshape((3,))
    T_Lid_cam = get_T_Lid_cam_Mocap(T_Mo_cam, T_Mo_Lid)
    R = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]
        ]) # /!\ Rotation Matrix around z-axis (OpenCv convention) applied in the Camera Frame
    T_Lid_cam = R @ T_Lid_cam

    for i in range(len(images)):
        idx_lidar = get_corresponding_pc_from_image(images[i], LiDAR_pcl_names)
        # print("Lidar choosen : ", LiDAR_pcl_names[idx_lidar])
        if type(LiDAR_data[idx_lidar][0]) == np.ndarray and type(LiDAR_data[idx_lidar][1])==np.ndarray:
            # print(f"Index :{i}/{len(images)}")
            display_points_in_image(os.path.join(images_path, images[i]), K, LiDAR_data[idx_lidar][0], LiDAR_data[idx_lidar][1], T_Lid_cam, cam_idx, dist_coeff, extension='.txt')
    cv.destroyAllWindows()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--path-to-dataset", help="Give the absolute root direction of the dataset.", type=str)
    parser.add_argument("--type", help="Whether you prefer to project a sequence or the calibration data. If sequence, give the sequence number in the argument sequence. If alibration, give the camera index you want to see in calibration argument"
                        , type=str, default="sequence")
    parser.add_argument("--sequence", help="The sequence name you want to visualize", default="sequence_00000", type=str)
    parser.add_argument("--cam_idx", help="The camera index from which you want to visualize the calibration data (202, 204, 205)", default="202", type=str)
    args = parser.parse_args()

    if args.type == "sequence":
        project_launch_dataset(args.path_to_dataset, args.sequence)
    elif args.type == "calibration":
        project_calibration_dataset(args.path_to_dataset, args.cam_idx)