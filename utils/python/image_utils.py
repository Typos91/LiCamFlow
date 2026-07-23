import numpy as np
import cv2 as cv
import glob
import json
import os

def get_list_ts(folder):
    """
    Get the list of timestamps corresponding to the list of images in folder with this format : timestamp.png/.bmp
    """
    images = []
    file_img = []
    for file in (os.listdir(folder)):
        if file.endswith('.png') or file.endswith('.bmp'):
            file_img.append(file)
            images.append(int(os.path.splitext(file)[0]))
    return sorted(images), sorted(file_img)

def rotate_images(image, angle_rot):
    """
    Angle rot should be either 'cv2.ROTATE_90_CLOCKWISE' or 'cv2.ROTATE_90_COUNTERCLOCKWISE'
    """
    rotated = cv.rotate(image, angle_rot)
    return rotated

def flip_images(folder_path:str, flip_code):
    """
    Use Flip code 0 to flip vertically, 1 to flip horizontally, -1 vertically and horizontally
    """
    for img in os.listdir(folder_path) :
        if img.endswith(".png") or img.endswith(".jpg"): 
            # Reading an image in default mode
            src = cv.imread(folder_path+img)

            # Using cv2.flip() method
            # Use Flip code 0 to flip vertically, -1 vertically and horizontally
            image = cv.flip(src, flip_code)

            # Displaying the image
            # cv2.imshow(window_name, image)
            # cv2.waitKey(0)
            cv.imwrite(folder_path+img, image)
            # print(f'Saved image {img}')

def resize_image(image, target_height):
    h, w = image.shape[:2]
    scale = target_height / h
    resized_img = cv.resize(image, (int(w * scale), target_height))
    return resized_img

def concatenate_image_horizontal(images_list):
    concatenated_image = np.hstack(images_list)
    return concatenated_image

def concatenate_image_vertical(images_list):
    concatenated_image = np.vstack(images_list)
    return concatenated_image

def readParams(json_file : str, cam_idx: int):
    """
    Read the camera parameters stored in the json config file for cameras (see README of the repo)
    """
    with open(json_file, "r") as f:
        data = json.load(f)
    cam = data[f"{cam_idx}"]
    T = np.array(cam["camera_pose"])
    K = np.array(cam["intrinsic_matrix"])
    res = np.array(cam["camera_resolution"])
    # dist_coef = np.array([0.0137242957240467, 0.003027000984213692, 0.001974943800789276, 0.003694050383102309, 0.0003651693407749373]) # Distortion coefficients to get from camera parameters file normally
    dist_coef = np.array(cam["dist_coeff"])
    return T, K, res, dist_coef



# ======================= Calibration functions ======================= #


def camera_calib(imgs_path : str, square_x=7, square_y=6, scale_obj_points=108, flip=None):
    """
    INPUT :
    - imgs_paths : Directory of all the images
    - square_x/y : Number of internal corners
    - scale_obj_points : size of one square in the chessboard
    - flip : how to flip the image (-1 --> horizontally and vertically, 0 --> Horizontally, 1 --> Vertically, None --> No flipping)
    """
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((square_x*square_y,3), np.float32)
    objp[:,:2] = np.mgrid[0:square_x,0:square_y].T.reshape(-1,2)
    objp *= scale_obj_points # Scale the object points to real size in mm
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(imgs_path)
    print("Found images:", len(images))
    for i in range(0, len(images), 5):
        # print("Image ", i)
        img = cv.imread(images[i])
        if flip != None:
            img = cv.flip(img, flip) # Flip the image vertically and horizontally
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # cv.imshow('img', gray)
        # cv.waitKey(500)
    
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (square_x,square_y), False)
        # print(f"Processing {images[i]}: Chessboard found: {ret}")
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
    
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
    
            # Draw and display the corners
            # cv.drawChessboardCorners(img, (square_x,square_y), corners2, ret)
            # cv.imshow('img', img)
            # cv.waitKey(50)
 
    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return ret, mtx, dist, rvecs, tvecs


def stereo_calibrate(mtx1, dist1, mtx2, dist2, frames_folder1, frames_folder2, square_x=7, square_y=6, scale_obj_points=108, flip1=None, flip2=None):
    """
    Description:
    The `stereo_calibrate` function performs stereo camera calibration using synchronized images from two cameras. It calculates the rotation (R) and translation (T) matrices to describe the spatial relationship between the cameras.

    Inputs:
    - `mtx1`, `dist1`, `mtx2`, `dist2`: Intrinsic matrices and distortion coefficients for both cameras.
    - `frames_folder1`, `frames_folder2`: Paths to synchronized image folders for camera 1 and 2.
    - `square_x`, `square_y`: Internal corners in the chessboard (default: 7x6).
    - `scale_obj_points`: Real-world size of one chessboard square (default: 108 mm).
    - `flip1`, `flip2`: Optional flipping modes for images from each camera.

    Outputs:
    - `R`: Rotation matrix from cam 2 to cam 1.
    - `T`: Translation vector from cam 2 to cam 1.

    Notes:
    - Images must be synchronized and of the same resolution.
    - Chessboard pattern must be visible in all images.
    """
    #read the synched frames
    c1_images_names = sorted(glob.glob(frames_folder1))
    c2_images_names = sorted(glob.glob(frames_folder2))
 
    c1_images = []
    c2_images = []
    for im1, im2 in zip(c1_images_names, c2_images_names):
        _im = cv.imread(im1, 1)
        if flip1 != None:
            _im = cv.flip(_im, flip1) # Flip the image vertically and horizontally
        c1_images.append(_im)
 
        _im = cv.imread(im2, 1)
        if flip2 != None:
            _im = cv.flip(_im, flip2) # Flip the image vertically and horizontally
        c2_images.append(_im)
 
    #change this if stereo calibration not good.
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
 
    #coordinates of squares in the checkerboard world space
    objp = np.zeros((square_x*square_y,3), np.float32)
    objp[:,:2] = np.mgrid[0:square_x,0:square_y].T.reshape(-1,2)
    objp *= scale_obj_points # Scale the object points to real size in mm
 
    #frame dimensions. Frames should be the same size.
    width = c1_images[0].shape[1]
    height = c1_images[0].shape[0]
 
    #Pixel coordinates of checkerboards
    imgpoints_left = [] # 2d points in image plane.
    imgpoints_right = []
 
    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space
 
    for frame1, frame2 in zip(c1_images, c2_images):
        gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
        c_ret1, corners1 = cv.findChessboardCorners(gray1, (square_x,square_y), False)
        c_ret2, corners2 = cv.findChessboardCorners(gray2, (square_x,square_y), False)
 
        if c_ret1 == True and c_ret2 == True:
            corners1 = cv.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
            corners2 = cv.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
 
            # cv.drawChessboardCorners(frame1, (square_x,square_y), corners1, c_ret1)
            # cv.imshow('img', frame1)
 
            # cv.drawChessboardCorners(frame2, (square_x,square_y), corners2, c_ret2)
            # cv.imshow('img2', frame2)
            # cv.waitKey(500)
 
            objpoints.append(objp)
            imgpoints_left.append(corners1)
            imgpoints_right.append(corners2)
 
    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx1, dist1,
                                                                 mtx2, dist2, (width, height), criteria = criteria, flags = stereocalibration_flags)
 
    # print(ret)
    return R, T