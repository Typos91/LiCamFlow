import os
import numpy as np
import cv2

"""
Description:
This script processes `.bmp` images from multiple camera folders, converts them to `.png` format, and organizes them into a structured directory. 
It aligns timestamps across cameras, creates output directories, and saves the converted images in the appropriate folders.

"""

def get_list_ts(folder):
    images = []
    file_img = []
    for file in (os.listdir(folder)):
        if file.endswith('.png') or file.endswith('.bmp'):
            file_img.append(file)
            images.append(int(os.path.splitext(file)[0]))
    return sorted(images), sorted(file_img)

def get_cut_lists(ts_List:list):
    ts0_max = np.max([ts_List[i][0] for i in range(0, len(ts_List))])
    ts_1max = np.min([ts_List[i][-1] for i in range(0, len(ts_List))])
    print("Start timestamp : ", ts0_max)
    print("End timestamp : ", ts_1max)
    IDX = []
    for i, ts_l in enumerate(ts_List):
        try :
            idx0 = next((j for j, ts in enumerate(ts_l) if ts >= ts0_max-1_000_000))
        except StopIteration:
            idx0 = 0
        try :
            idx_1 = next((j for j, ts in enumerate(ts_l) if ts >= ts_1max-1_000_000))
        except StopIteration:
            idx_1 = -1
        ts_List[i] = ts_l[idx0 : idx_1]    
        IDX.append([idx0, idx_1])
    return ts_List, IDX

def get_ts_List(launch_folder:str):
    folders = sorted([ name for name in os.listdir(launch_folder) if os.path.isdir(os.path.join(launch_folder, name)) and name.startswith("cam_")])
    # print(f"Folders in {launch_folder} : ", folders)
    ts_List = []
    ts_files_List = []
    for folder in folders:
        ts_list, file_list = get_list_ts(launch_folder+folder)
        ts_List.append(ts_list)
        ts_files_List.append(file_list)
    return ts_List, ts_files_List, folders

def create_png_dirs(launch_folder:str, folders:list):
    if len(folders) > 0:
        try : 
            os.makedirs(launch_folder+"/png_images", exist_ok=True)
        except : 
            print("png_images file not created, quitting")
            return False
        try:
            for folder in folders:
                os.makedirs(launch_folder+"/png_images/"+folder, exist_ok=True)
        except: 
            print("Camera folders not created, quitting")
            return False
        # print("All folders created without error.")
        return True
    print("No folder to create.")
    return False

def list_launch_folders(folder_path:str):
    folders = [ name for name in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, name)) and name.startswith("launch_")]
    return sorted(folders)

def copy_bpm2png(cam_directon:str, bmp_name:str, png_cam_direction:str):
    ts_name = bmp_name.split('.')[0]
    bmpIm = cv2.imread(cam_directon+bmp_name, cv2.IMREAD_GRAYSCALE)
    if bmpIm is None:
        print("Couldn't bmp read image")
        return False
    else:
        png_name = png_cam_direction+ts_name+'.png'
        cv2.imwrite(png_name, bmpIm)
        return True


if __name__ == "__main__":
    path_to_dataset = "path/to/dataset"
    data_raw_path = f"{path_to_dataset}/dataset_raw/"
    launch_folders = list_launch_folders(data_raw_path)
    for i, launch_folder in enumerate(launch_folders[:]) :
        print('=============================================')
        print(f"Launch : {i+1}/{len(launch_folders[:])}")
        ts_List, ts_files_List, folders = get_ts_List(data_raw_path+launch_folder+'/')
        if create_png_dirs(data_raw_path+launch_folder, folders):
            print("Image folders created")
            ts_cut_List, Idx = get_cut_lists(ts_List)
            ts_files_cut_List = [ts_list[Idx[i][0]:Idx[i][1]] for i, ts_list in enumerate(ts_files_List)]
            for j, folder in enumerate(folders) :
                print(f"Camera : {folder}")
                full_path = data_raw_path+launch_folder+'/'+folder+'/'
                new_path = data_raw_path+launch_folder+'/png_images/'+folder+'/'
                lost_cnt = 0
                for ts_img in ts_files_cut_List[j]:
                    if not copy_bpm2png(full_path, ts_img, new_path):
                        lost_cnt+=1
                        print("Image not copied : ", lost_cnt)
