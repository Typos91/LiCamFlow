import os
import subprocess
import numpy as np
import json


"""
Calling the smoke_labelling c++ package for all the launch files. Make sure that the package is previously built.
"""


def list_folders(folder_path:str):
    folders = [ name for name in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, name)) and name.startswith("launch_wind_")]
    print(sorted(folders))
    return sorted(folders)

def list_fullcloud(folder:str):
    folder_full = [ name for name in os.listdir(folder) if os.path.isdir(os.path.join(folder, name)) and name.startswith("full_cloud")]
    # print(folder_full)
    return folder_full

def run_smoke_labelling(config_path:str):
    result = subprocess.run(
        ["./utils/cpp/smoke_labelling/build/smoke_labelling", config_path],
        capture_output=True,
        text=True
    )
    return result

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--path-to-dataset", help="Give the absolute root direction of the dataset.", type=str)
    parser.add_argument("--path-to-config", help="Give the path to the json config file for the cpp packages.", type=str)
    parser.add_argument("--angle-max", help="Horizontal FOV/2 you want to select in deg (max 180°)", type=int, default=45)
    parser.add_argument("--last-ref-frame", help="Number of frame to build the reference OCTREE map", type=int, default=30)
    parser.add_argument("--octree-resolution", help="Octree map resolution", type=float, default=0.3)
    parser.add_argument("--extension-type", help="Wanted extension to save pointclouds ('.pcd' or '.txt')", type=str, default=".pcd")
    parser.add_argument('--box-dimensions', nargs='*', default=[5.0, 5.0, 6.0, 1.0], help="Clipping box dimmension to reduce outliers impact.")
    args = parser.parse_args() 


    path_to_dataset = args.path_to_dataset
    arg_file = args.path_to_config
    with open(arg_file, "r") as f:
        data = json.load(f)
    folder_path = f"{path_to_dataset}/"
    direction = "/lidars/full_pointclouds/"
    folders = list_folders(folder_path)
    for folder in folders :
        try :
            folder_full = list_fullcloud(folder_path+folder+direction)[0]
            os.makedirs(os.path.join(folder_path,folder,direction,"full_labelized"))
            os.makedirs(os.path.join(folder_path,folder,direction,"full_extracted"))
            output_dir = os.path.join(folder_path,folder,direction,"full_labelized/")
            output_ext = os.path.join(folder_path,folder,direction,"full_extracted/")
            data["data_directions"]["input"]["path"] = os.path.join(folder_path,folder,direction,folder_full,"/")
            data["data_directions"]["output"]["path"] = output_dir
            data["data_directions"]["output_extracted"]["path"] = output_ext
            # For windy data, box dimensions (5.0, 5.0, 6.0) otherwise 1m wide is enough
            data["data_args"]["box_dimensions"] = args.box_dimension
            data["data_args"]["last_ref_frame"] = args.last_ref_frame
            data["data_args"]["octree_resolution"] = args.octree_resolution
            data["data_args"]["angle_max"] = args.angle_max
            with open(arg_file, "w") as f:
                json.dump(data, f, indent=4)
            print(f"Executing smoke extraction for launch {folder}")
            result = run_smoke_labelling(arg_file)
            if result.returncode == 0:
                print(f"Successfully processed {folder}")
            else:
                print(f"Error processing {folder}: {result.stderr}")
        except :
            print(f"No full cloud generated for the folder {folder}")