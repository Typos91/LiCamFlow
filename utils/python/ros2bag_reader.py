#made by Jamiiiiiila and Myriiiiiam (no)
import os
import math
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from custom_interfaces.msg import MoCapPoses
from collections import defaultdict



def read_ros2_bag(bag_path, output_dir, prefix="txt", name="ts", smoke_pose = [0, 0, 0]):
    """
    Read a ros2 bag of point clouds, and generates txt or pcd files containing the pointclouds.\n
    INPUTS :\n
        - bag_path : Directory of the bag file
        - output_dir : Directory where to save the generated files
        - prefix : Type of file to generate (txt or pcd)
        - ts : Type of naming for the files (timestamps 'ts' or count 'cnt')
    """
    os.makedirs(output_dir, exist_ok=True)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    topic_counters = defaultdict(int)  # compteur par topic


    while reader.has_next():
        # print("Reading data")
        topic, data, t = reader.read_next()
        try :
            # print("Trying to read PC2")
            msg = deserialize_message(data, PointCloud2)
        except Exception as e:
            # print(f"Message is not of type Pointcloud2 : {e}. Trying with MoCapPose message type.")
            try :
                # print("Trying to read Mocap")
                msg = deserialize_message(data, MoCapPoses)
            except Exception as e:
                print(f"Error when deserialising MoCapPoses : {e}")
                exit(-1)



        if isinstance(msg, PointCloud2):
            # Création du nom de fichier basé sur le timestamp
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            topic_suffix = topic.strip('/').split('/')[-1]
            topic_dir = os.path.join(output_dir, topic_suffix)
            os.makedirs(topic_dir, exist_ok=True)
            # Récupérer compteur pour ce topic
            count = topic_counters[topic]
            if prefix=="txt":
                if name=="ts":
                    filename = os.path.join(topic_dir, f"{timestamp_ns}.txt")
                elif name=="cnt":
                    filename = os.path.join(topic_dir, f"{100000+count}.txt") # For the automatic Livox calibration
                save_pointcloud2_to_txt(msg, filename, smoke_pose = smoke_pose)
                # print(f"Saved: {filename}")
                topic_counters[topic] += 1
            elif prefix=="pcd":
                if name=="ts":
                    filename = os.path.join(topic_dir, f"{timestamp_ns}.pcd")
                elif name=="cnt":
                    filename = os.path.join(topic_dir, f"{100000+count}.pcd") # For the automatic Livox calibration
                save_pointcloud2_to_pcd(msg, filename, smoke_pose = smoke_pose)
                # print(f"Saved: {filename}")
                topic_counters[topic] += 1
            elif prefix=="npy":
                if name=="ts":
                    filename = os.path.join(topic_dir, f"{timestamp_ns}.pcd")
                elif name=="cnt":
                    filename = os.path.join(topic_dir, f"{100000+count}.pcd") # For the automatic Livox calibration
                save_pointcloud2_to_npy(msg, filename, smoke_pose = smoke_pose)
                # print(f"Saved: {filename}")
                topic_counters[topic] += 1
            
        elif isinstance(msg, MoCapPoses):
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            topic_suffix = topic.strip('/').split('/')[-1]
            topic_dir = os.path.join(output_dir, topic_suffix)
            os.makedirs(topic_dir, exist_ok=True)
            count = topic_counters[topic]
            filename = os.path.join(topic_dir, f"{timestamp_ns}.txt")
            # print("Saving Mocap")
            save_mocapposes_to_txt(msg, filename)
            # print(f"Saved: {filename}")
            topic_counters[topic] += 1
        else :
            raise Exception("Wrong type of file to generate. Choose between '.txt' or '.pcd'.")


def save_pointcloud2_to_txt(cloud_msg, filename, smoke_pose = [0, 0, 0]):
    field_names = [f.name for f in cloud_msg.fields]

    # Vérifie présence de x, y, z
    if not all(k in field_names for k in ('x', 'y', 'z')):
        print(f"Skipping point cloud without x/y/z: {filename}")
        return

    with open(filename, 'w') as f:
        f.write(','.join(field_names) + '\n')
        for point in point_cloud2.read_points(cloud_msg, field_names=field_names, skip_nans=True):
            # Re-center smoke 
            point[field_names.index('x')] -= smoke_pose[0]
            point[field_names.index('y')] -= smoke_pose[1]
            point[field_names.index('z')] -= smoke_pose[2]
            x, y, z = point[field_names.index('x')], point[field_names.index('y')], point[field_names.index('z')]
            norm = math.sqrt(x ** 2 + y ** 2 + z ** 2)
            if norm > 0 and norm < 13:
                # print("Writing point")
                f.write(','.join(map(str, point)) + '\n')

def save_pointcloud2_to_npy(cloud_msg, filename, smoke_pose=[0, 0, 0]):
    field_names = [f.name for f in cloud_msg.fields]

    # Check for presence of x, y, z
    if not all(k in field_names for k in ('x', 'y', 'z')):
        print(f"Skipping point cloud without x/y/z: {filename}")
        return

    points = []
    for point in point_cloud2.read_points(cloud_msg, field_names=field_names, skip_nans=True):
        # Re-center smoke
        point_list = list(point)
        point_list[field_names.index('x')] -= smoke_pose[0]
        point_list[field_names.index('y')] -= smoke_pose[1]
        point_list[field_names.index('z')] -= smoke_pose[2]

        x, y, z = point_list[field_names.index('x')], point_list[field_names.index('y')], point_list[field_names.index('z')]
        norm = math.sqrt(x ** 2 + y ** 2 + z ** 2)

        if norm > 0 and norm < 13:
            points.append(point_list)

    # Convert to numpy array and save
    if points:
        points_array = np.array(points)
        np.save(filename, points_array)
    else:
        print(f"No points to save for: {filename}")

def save_mocapposes_to_txt(msg, filename):
    fields = ["id", "x", "y", "z", "qx", "qy", "qz", "qw"]
    with open(filename, 'w') as f:
        f.write(','.join(fields) + '\n')
        for id, pose in zip(msg.ids, msg.poses):
            id_txt = id
            x_txt, y_txt, z_txt = pose.position.x, pose.position.y, pose.position.z
            qx_txt, qy_txt, qz_txt, qw_txt = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w 
            point = np.array([id_txt, x_txt, y_txt, z_txt, qx_txt, qy_txt, qz_txt, qw_txt])
            f.write(','.join(map(str, point)) + '\n')


def save_pointcloud2_to_pcd(cloud_msg, filename, smoke_pose = [0, 0, 0]):
    field_names = [f.name for f in cloud_msg.fields]

    # Vérifie présence de x, y, z
    if not all(k in field_names for k in ('x', 'y', 'z')):
        print(f"Skipping point cloud without x/y/z: {filename}")
        return

    # Liste pour les points filtrés
    filtered_points = []

    for point in point_cloud2.read_points(cloud_msg, field_names=field_names, skip_nans=True):
        # Re-center smoke 
        point[field_names.index('x')] -= smoke_pose[0]
        point[field_names.index('y')] -= smoke_pose[1]
        point[field_names.index('z')] -= smoke_pose[2]

        x = point[field_names.index('x')]
        y = point[field_names.index('y')]
        z = point[field_names.index('z')]

        norm = math.sqrt(x ** 2 + y ** 2 + z ** 2)

        if norm > 0 and norm < 13:
            filtered_points.append(point)

    if not filtered_points:
        print(f"No valid points found for {filename}, skipping.")
        return

    # Déduire les informations pour le header PCD
    count = len(filtered_points)
    field_line = 'FIELDS ' + ' '.join(field_names)
    size_line = 'SIZE ' + ' '.join(['4'] * len(field_names))
    type_line = 'TYPE ' + ' '.join(['F'] * len(field_names))
    count_line = 'COUNT ' + ' '.join(['1'] * len(field_names))

    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        field_line,
        size_line,
        type_line,
        count_line,
        f"WIDTH {count}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {count}",
        "DATA ascii"
    ]

    with open(filename, 'w') as f:
        f.write('\n'.join(header) + '\n')
        for point in filtered_points:
            line = ' '.join(str(val) for val in point)
            f.write(line + '\n')

def list_folders(folder_path:str):
    folders = [ name for name in os.listdir(folder_path) if os.path.isdir(os.path.join(folder_path, name)) and name.startswith("launch_")]
    print(sorted(folders))
    return sorted(folders)

def list_bags(bag_path:str):
    bags = [bag for bag in os.listdir(bag_path) if bag.startswith("bag_")]
    print(sorted(bags))
    return sorted(bags)

def main_list():
    smoke_pose = [0, 0, 0]
    path_to_dataset = "path/to/dataset"
    folder_path = f"{path_to_dataset}/dataset_raw/"
    folders = list_folders(folder_path)
    for folder in folders :
        bag_path = folder_path + folder + "/lidars/"
        bags = list_bags(bag_path)
        print(len(bags) == len(os.listdir(bag_path)))
        if len(bags) == len(os.listdir(bag_path)):
            if len(bags) == 2:
                os.makedirs(bag_path+"/full_pointclouds_txt", exist_ok=True)
                output_full = bag_path+"/full_pointclouds_txt"
                read_ros2_bag(bag_path + bags[0], output_full, prefix="txt", name="ts", smoke_pose=smoke_pose)
                # ---------------
                os.makedirs(bag_path+"/pointclouds", exist_ok=True)
                output_dir = bag_path+"/pointclouds"
                read_ros2_bag(bag_path + bags[1], output_dir, prefix="txt", name="ts", smoke_pose=smoke_pose)

            elif len(bags) == 1:
                os.makedirs(bag_path+"/pointclouds", exist_ok=True)
                output_dir = bag_path+"/pointclouds"
                read_ros2_bag(bag_path + bags[0], output_dir, prefix="txt", name="ts", smoke_pose=smoke_pose)

        else :
            os.makedirs(bag_path+"/pointclouds", exist_ok=True)
            output_dir = bag_path+"/pointclouds"
            read_ros2_bag(bag_path + bags[1], output_dir, prefix="txt", name="ts", smoke_pose=smoke_pose)

def main_alone():
    smoke_pose = [0, 0, 0]
    path_to_dataset = "path/to/dataset"
    sequence_to_deserialize = "launch_2026-02-23_15-14"
    bag_path = f"{path_to_dataset}/dataset_raw/{sequence_to_deserialize}/lidars/"
    os.makedirs(bag_path+"/pointclouds", exist_ok=True)
    output_dir = bag_path+"/pointclouds"
    read_ros2_bag(bag_path + "bag_2026-02-23_15-14_lidar/", output_dir, prefix="txt", name="ts", smoke_pose=smoke_pose)
# Exemple d'utilisation
if __name__ == '__main__':
    main_alone()
