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

def run_smoke_labelling():
    result = subprocess.run(
        ["./lidar/utils/cpp/smoke_labelling/build/smoke_labelling"],
        capture_output=True,
        text=True
    )
    return result

if __name__ == '__main__':
    path_to_dataset = "path/to/dataset/"
    arg_file = "./lidar/utils/cpp/smoke_labelling/config/arguments.json"
    with open(arg_file, "r") as f:
        data = json.load(f)
    folder_path = f"{path_to_dataset}/dataset_raw/"
    direction = "/lidars/full_pointclouds_txt/"
    folders = list_folders(folder_path)
    for folder in folders :
        try :
            folder_full = list_fullcloud(folder_path+folder+direction)[0]
            os.makedirs(folder_path+folder+direction+"full_labelized")
            os.makedirs(folder_path+folder+direction+"full_extracted")
            output_dir = folder_path+folder+direction+"full_labelized/"
            output_ext = folder_path+folder+direction+"full_extracted/"
            data["data_directions"]["input"]["path"] = folder_path+folder+direction+folder_full+"/"
            data["data_directions"]["output"]["path"] = output_dir
            data["data_directions"]["output_extracted"]["path"] = output_ext
            # For windy data, box dimensions (5.0, 5.0, 6.0) otherwise 1m wide is enough
            data["data_args"]["box_dimensions"] = [5.0, 5.0, 6.0, 1.0]
            data["data_args"]["last_ref_frame"] = 20
            data["data_args"]["octree_resolution"] = 0.3
            with open(arg_file, "w") as f:
                json.dump(data, f, indent=4)
            print(f"Executing smoke extraction for launch {folder}")
            result = run_smoke_labelling()
            if result.returncode == 0:
                print(f"Successfully processed {folder}")
            else:
                print(f"Error processing {folder}: {result.stderr}")
        except :
            print(f"No full cloud generated for the folder {folder}")