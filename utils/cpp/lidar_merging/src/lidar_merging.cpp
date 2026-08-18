#include "lidar_merging.hpp"


using namespace std;
using json = nlohmann::json;
namespace fs = std::filesystem;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

bool allValuesEqual(const std::map<int, int>& m) {
    if (m.empty()) return true; // Consider empty map as "all equal"
    auto it = m.begin();
    int firstValue = it->second;
    for (const auto& pair : m) {
        if (pair.second != firstValue) {
            return false;
        }
    }
    return true;
}

void savePointCloudToTxt(
    const PointCloudXYZII::Ptr& cloud,
    const std::string& directory,
    const std::string& filename
) {
    // Create the full file path
    std::string fullPath = directory + "/" + filename + ".txt";

    // Open the file for writing
    std::ofstream outFile(fullPath);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << fullPath << std::endl;
        return;
    }
    outFile << "x" << "," << "y" << "," << "z" << ","
                << "intensity" << "," << "id" << "\n";
    // Write each point in the specified format
    for (const auto& point : cloud->points) {
        outFile << point.x << "," << point.y << "," << point.z << ","
                << point.intensity << "," << point.id << "\n";
    }

    // Close the file
    outFile.close();
    // std::cout << "Point cloud saved to: " << fullPath << std::endl;
}

PointMoCap::Ptr loadMoCapPose(fs::path file_path, int id_mocap, int id_lidar){
    /*Extract the Mocap pose (x, y, z, quat, id_lidar) from a txt file*/
    PointMoCap::Ptr mocapPoint(new PointMoCap());
    // std::cout << "Reading MoCap file : " << file_path.c_str() << std::endl;

    // Opening file and making sure to start at the begining
    std::ifstream file(file_path);
    file.clear();
    file.seekg(0);
    // std::cout << "File is open : " << file.is_open() << std::endl;
    // Pass the first line (id,x,y,z,qx,qy,qz,qw)
    std::string str; 
    std::getline(file, str);
    // std::cout << "Reading first line of MoCap file : " << str << std::endl;

    // Get the point with the corresponding mocap_id
    while (std::getline(file, str)) // Read the rest of the file
    {     
        // std::cout << "Line : " << str << std::endl;    
        // Use separator to read parts of the line
        std::istringstream line_stream(str);
        std::string token;
        // Find the line corresponding to the right mocap id
        // Point format: id(id_mocap),x,y,z,qx,qy,qz,qw
        if(std::getline(line_stream, token, ',')){
            if (std::stoi(token)==id_mocap){
                // std::cout << "Reading MoCap File" << std::endl;
                mocapPoint->id = id_lidar;
                if (std::getline(line_stream, token, ',')) mocapPoint->point.x = std::stof(token);
                if (std::getline(line_stream, token, ',')) mocapPoint->point.y = std::stof(token);
                if (std::getline(line_stream, token, ',')) mocapPoint->point.z = std::stof(token);
                if (std::getline(line_stream, token, ',')) mocapPoint->quat.x = std::stof(token);
                if (std::getline(line_stream, token, ',')) mocapPoint->quat.y = std::stof(token);
                if (std::getline(line_stream, token, ',')) mocapPoint->quat.z = std::stof(token);
                if (std::getline(line_stream, token, ',')) mocapPoint->quat.w = std::stof(token);
            }
            else{
                continue;
            }
        }
    }
    return mocapPoint;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(fs::path file_path){
    /*Extract the pointcloud (X, Y, Z, Intensity) from a txt file*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>()); // Initialisation of the pointcloud 

    // First we compute the number of points
    std::string str; 
    int count_lines =0;

    // Counting the number of points
    std::ifstream file(file_path);
    std::getline(file, str); // Read first line
    while (std::getline(file, str)) // Read the rest of the file
    {
        count_lines ++; // Get the exact number of points
    }
    cloud->width = count_lines;
    cloud->height = 1;
    cloud->is_dense = true;
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> points(count_lines); // Where points will be stored
    file.clear();
    file.seekg(0);
    std::getline(file, str); // Read first line
    int count_points =0;
    while (std::getline(file, str)) // Read the rest of the file
    {
        // Use separator to read parts of the line
        std::istringstream line_stream(str);
        std::string token;
        pcl::PointXYZI point;

        // Point format: x,y,z,intensity
        if (std::getline(line_stream, token, ',')) point.x = std::stof(token);
        if (std::getline(line_stream, token, ',')) point.y = std::stof(token);
        if (std::getline(line_stream, token, ',')) point.z = std::stof(token);
        if (std::getline(line_stream, token, ',')) point.intensity = std::stof(token);
        // cout << point <<endl;
        points[count_points] = point;
        count_points ++;
    }
    // Assign the points and complete the pointcloud informations
    cloud->points = points;

    return cloud;
}


int get_lidars_id(fs::path entry){
    std::string dirName = entry.filename().string();
    // Split the string by '_'
    std::istringstream iss(dirName);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(iss, token, '_')) {
        tokens.push_back(token);
    }

    std::string lidar_id = tokens[tokens.size() - 1];
    return stoi(lidar_id);
}

int main(int argc, char * argv[]){
    // Read config file 
    std::string config_dir = "/home/gperez/Documents/lidar-for-gaussian-physics/lidar/utils/cpp/lidar_merging/config/arguments.json";
    std::ifstream config_file(config_dir);
    if (!config_file.is_open()) {
        std::cerr << "Erreur : impossible d'ouvrir le fichier de configuration." << std::endl;
        return 1;
    }
    json config = json::parse(config_file);

    // Define the variables

    fs::path folder_path = config["data_directions"]["input"]["path"];
    fs::path mocap_path = "mocap_poses";
    fs::path saving_path = config["data_directions"]["output"]["path"];
    std::map<std::string, int> map_ids = config["data_args"]["map_ids"];
    int ref_id = config["data_args"]["ref_id"];
    std::cout << "Description des paramètres : " << config["data_args"]["description"] << std::endl;

    std::map<int, vector<fs::path>> pointclouds;
    vector<fs::path> mocap_files;
    int count = 0; // Number of lidars  
    fs::path extension(config["data_args"]["extension_type"]);

    /*-----------------------------------Get the files--------------------------------------------------*/
    // First we count the number of files, before storing them inside a vector
    for (const auto & entry : fs::directory_iterator(folder_path)){
        std::cout << "File : " << entry << std::endl;
        if(!(entry.is_directory() && entry.path().filename().string().rfind("lidar_", 0))){
            int lidar_id = get_lidars_id(entry.path());
            std::cout << "Lidar Id : " << lidar_id << std::endl;
            std::cout << "Mocap Id : " << map_ids[to_string(lidar_id)] << std::endl;
            fs::path lidar_path = "lidar_192_168_2_"+to_string(lidar_id);
            // Get the vectors of pointclouds for each lidar
            vector<fs::path> txt_pointclouds; 
            int k = 0;
            for (const auto & pc_entry : fs::directory_iterator(folder_path/lidar_path)){
                txt_pointclouds.push_back(pc_entry.path().filename()); // Store the files name inside vector
                // cout << txt_pointclouds[k] << endl;
                k ++;
            }
            // Sorting the vector so that the files are ordered according to their timestamps
            sort(txt_pointclouds.begin(), txt_pointclouds.end());
            pointclouds[lidar_id] = txt_pointclouds;
            count ++;
        }
        else if(!(entry.is_directory() && entry.path().filename().string().rfind("mocap", 0))){
            // std::cout << "Saving MoCap File" << std::endl;
            for (const auto & pc_entry : fs::directory_iterator(folder_path/mocap_path)){
                // std::cout << "Saving MoCap File : " << pc_entry.path().c_str() << std::endl;
                mocap_files.push_back(pc_entry.path().filename()); // Store the files name inside vector
            }
        }
    }
    std::map<int, int> vector_sizes({{108, 0}, {178, 0}, {152, 0}});
    for (auto it = pointclouds.begin(); it != pointclouds.end(); ++it){
        vector_sizes[it->first] = it->second.size();
        // std::cout<< "Size of Lidar " << it->first << ": " << it->second.size() <<std::endl;
    }
    if (!allValuesEqual(vector_sizes)){
        std::cerr << "Not the same number of pointclouds per lidar. Please check " << folder_path.c_str() << std::endl;
        return 1; 
    }

    // Get the mocap poses for each lidar, and the reference 
    PointMoCap::Ptr ref_pose = loadMoCapPose(folder_path/mocap_path/mocap_files[10], map_ids[to_string(ref_id)], ref_id);
    Eigen::Affine3d ref_T = ref_pose->toAffine3d();
    std::map<int, PointMoCap::Ptr> mocap_poses;
    for (auto it = pointclouds.begin(); it != pointclouds.end(); ++it){
        // std::cout << "LiDARs : "  << it->first << std::endl;
        mocap_poses[it->first] = loadMoCapPose(folder_path/mocap_path/mocap_files[10], map_ids[to_string(it->first)], it->first);
    } 
    
    /*-----------------------------------------------Apply transformations--------------------------------------------------*/

    // Get everything in the reference frame
    std::map<int, Eigen::Affine3d> ref_frame_lidar_poses;
    for (auto it = mocap_poses.begin(); it != mocap_poses.end(); ++it){
        // std::cout << "Pose " << it->first << " : ";
        it->second->print();
        Eigen::Affine3d pose = it->second->toAffine3d(); 
        std::cout << "Affine pose:"<< std::endl << pose.matrix() << std::endl;
        ref_frame_lidar_poses[it->first] = ref_T.inverse()*pose;
        // printAffine3d(pose);
    }
    /*-----------------------------------------------Label and Transform the pointclouds--------------------------------------------------*/

    // std::vector<PointCloudXYZII::Ptr> pointclouds_with_id(pointclouds[108].size()); // All folders have the same number of pointclouds
    std::vector<long long int> avg_TS;
    PointCloudXYZII::Ptr cloudOut(new PointCloudXYZII);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZI>());
    PointCloudXYZII::Ptr full_cloud(new PointCloudXYZII());
    for (int i=0; i<pointclouds[108].size(); i++){ 
        // Get the average timestamp
        int count = 0;
        long long int avg_ts = 0;
        for (auto it = pointclouds.begin(); it != pointclouds.end(); ++it){
            // Get Name file and direction
            int lidar_id = it->first;
            fs::path lidar_path = "lidar_192_168_2_"+to_string(lidar_id);
            std::string name_file = it->second[i];
            fs::path file_path = name_file;
            fs::path full_path = folder_path/lidar_path/file_path;
            // Extract timestamp from file name
            size_t dotPos = name_file.find('.');
            std::string numberStr = name_file.substr(0, dotPos);
            long long int ts = std::stoll(numberStr);
            avg_ts += ts;
            // std::cout << "Timestamp extracted" << std::endl;
            // Load PointCloud, apply transform and add Id
            pcl = loadPointCloud(full_path);
            pcl::transformPointCloud(*pcl, *pcl, ref_frame_lidar_poses[it->first]);
            // std::cout << "Point Cloud loaded" << std::endl;
            // Copy and convert to PointCloudXYZII
            CopyPointXYZIToXYZII(pcl, cloudOut, lidar_id);
            // std::cout << "Point Cloud Converted. Cloud Out size : " << cloudOut->size() << std::endl;
            *full_cloud += *cloudOut;
            // std::cout << "Point Cloud Added" << std::endl;
            cloudOut.reset(new PointCloudXYZII());
            pcl.reset(new pcl::PointCloud<pcl::PointXYZI>());
            count ++;
        } 
        avg_ts/=3;
        std::string cloud_name = to_string(avg_ts);
        std::string full_dir = folder_path.c_str()+std::string("/merged_lidars");
        if (std::filesystem::create_directory(full_dir)) std::cout <<" Direction created "<< std::endl;
        savePointCloudToTxt(full_cloud, full_dir, cloud_name);
        full_cloud.reset(new PointCloudXYZII());
        // std::cout << "Avg timestamp : " << avg_ts << std::endl;
    }
    return 0;
}