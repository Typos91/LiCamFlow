# LiDAR Merging Utility

This program processes and merges LiDAR point cloud data from multiple sensors, aligning them to a common reference frame using motion capture (MoCap) data. The merged point clouds are saved as `.txt` files for further analysis. Each pointclouds has a label ***lidar_id*** enabling to know wich point clorresponds to which LiDAR.

## Workflow

1. **Configuration**:
   - Reads configuration from `arguments.json` to define input/output paths, LiDAR IDs, and other parameters.

2. **File Organization**:
   - Scans the input directory for LiDAR and MoCap files.
   - Groups point clouds by LiDAR ID.

3. **Reference Frame Alignment**:
   - Loads the reference MoCap pose and computes transformations for each LiDAR.

4. **Point Cloud Processing**:
   - For each timestamp:
     - Loads and transforms point clouds to the reference frame.
     - Merges transformed point clouds.
     - Saves the merged point cloud with the average timestamp as the filename.

## Key Functions

- **`loadPointCloud`**: Reads a point cloud from a `.txt` file.
- **`loadMoCapPose`**: Extracts MoCap pose data for a specific LiDAR.
- **`savePointCloudToTxt`**: Saves a point cloud to a `.txt` file.
- **`allValuesEqual`**: Checks if all LiDARs have the same number of point clouds.
- **`get_lidars_id`**: Extracts the LiDAR ID from a directory name.

## Input/Output

- **Input**:
  - Point cloud files in `.txt` format.
  - MoCap pose files in `.txt` format.
  - Configuration file: `arguments.json`.

- **Output**:
  - Merged point clouds saved in the `merged_lidars` directory.

## Dependencies

- **Libraries**:
  - [PCL (Point Cloud Library)](https://pointclouds.org/)
  - [Eigen](https://eigen.tuxfamily.org/)
  - [nlohmann/json](https://github.com/nlohmann/json)

## Example Configuration (`arguments.json`)

```json
{
  "data_directions": {
    "input": { "path": "/path/to/input" },
    "output": { "path": "/path/to/output" }
  },
  "data_args": {
    "map_ids": {"108":202, "178":204, "152":205, "0":206},
    "ref_id": 0,
    "extension_type": ".txt",
    "description": "LiDAR merging configuration"
  }
}

## Get started

### Building the program

In order to build this program:

```bash
cd lidar_merging
mkdir build && cd build
cmake ..
make
```

### Launching

You can modify the parameters in the [config file](./config/arguments.json).

```bash
cd lidar_merging/build/
./lidar_merging
```
