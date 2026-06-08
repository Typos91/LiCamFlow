#include "smoke_labelling.hpp"


using namespace std;
using json = nlohmann::json;
namespace fs = std::filesystem;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


pcl::PointCloud<pcl::PointXYZI>::Ptr referencePointCloud(int ts1, int ts2, fs::path folder_path, vector<fs::path> &txt_pointclouds){
    /*
    INPUTS: ts1, ts2, folder_path, txt_pointclouds. ts1 and ts2 are the number of the 
    files delimiting the period where there is no smoke in the pointcloud.

    This function creates a reference PointCloud, which will be used to detect
    the smoke.
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr ref_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // Initialisation of the reference pointcloud 

    // First we compute the number of points
    std::string str; 
    int count_lines =0;
    // Counting the number of points
    for (int i=ts1; i < ts2; i++){
        fs::path file_path = folder_path / txt_pointclouds[i];
        std::ifstream file(file_path);
        std::getline(file, str); // Read first line
        while (std::getline(file, str)) // Read the rest of the file
        {
            count_lines ++; // Get the exact number of points
        }
    }
    ref_cloud->width = count_lines;
    ref_cloud->height = 1;
    ref_cloud->is_dense = true;
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> points(count_lines); // Where points will be stored
    
    // Storing the points
    int count_points=0;
    for (int i=ts1; i < ts2; i++){
        fs::path file_path = folder_path / txt_pointclouds[i];
        std::ifstream file(file_path);
        std::getline(file, str); // Read first line
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
    }

    // Assign the points and complete the pointcloud informations
    ref_cloud->points = points;

    return ref_cloud;
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

Eigen::Vector4f min_box(Eigen::Vector4f center_box, Eigen::Vector4f dimension_box){
    float tr_x = float(dimension_box[0])/float(2.0);  // Translation to get to the 'x' edge
    float tr_y = float(dimension_box[1])/float(2.0);  // Translation to get to the 'y' edge
    float tr_z = float(dimension_box[2]);      // Translation to get to the 'z' edge, supposing the value of center_box['z'] = 0.0

    Eigen::Vector4f minPoint(center_box[0] - tr_x, center_box[1] - tr_y, 0.0, 1.0);
    std::cout << "Min point of the box : " << "x = " << minPoint[0] << ";y = " << minPoint[1] << ";z = " << minPoint[2] << std::endl;

    return minPoint;
}

Eigen::Vector4f max_box(Eigen::Vector4f center_box, Eigen::Vector4f dimension_box){
    float tr_x = float(dimension_box[0])/float(2.0);  // Translation to get to the 'x' edge
    float tr_y = float(dimension_box[1])/float(2.0);  // Translation to get to the 'y' edge
    float tr_z = float(dimension_box[2]);      // Translation to get to the 'z' edge, supposing the value of center_box['z'] = 0.0

    Eigen::Vector4f maxPoint(center_box[0] + tr_x, center_box[1] + tr_y, center_box[2] + tr_z, 1.0);
    std::cout << "Max point of the box : " << "x = " << maxPoint[0] << ";y = " << maxPoint[1] << ";z = " << maxPoint[2] << std::endl;

    return maxPoint;
}


int main(int argc, char * argv[]){
    // Read config file 
    std::ifstream config_file(argv[0]);
    if (!config_file.is_open()) {
        std::cerr << "Erreur : impossible d'ouvrir le fichier de configuration." << std::endl;
        return 1;
    }
    json config = json::parse(config_file);

    // Define the variables

    float angle_max = config["data_args"]["angle_max"]; // Max angle where the smoke is supposed to be  (in degree)
    int ts1 = 0; int ts2 = config["data_args"]["last_ref_frame"]; // Numero of the files delimiting the reference poincloud
    fs::path folder_path = config["data_directions"]["input"]["path"];
    fs::path saving_path = config["data_directions"]["output"]["path"];
    fs::path saving_path_extracted = config["data_directions"]["output_extracted"]["path"];
    float resolution = config["data_args"]["octree_resolution"];
    std::vector<float> center = config["data_args"]["smoke_pose"].get<std::vector<float>>();
    Eigen::Vector4f box_center(center.data());
    // std::cout << "Box center : " << "x = " << box_center[0] << ";y = " << box_center[1] << ";z = " << box_center[2] << std::endl;
    std::vector<float> dims = config["data_args"]["box_dimensions"].get<std::vector<float>>();
    Eigen::Vector4f box_dim(dims.data());
    // std::cout << "Box dimensions : " << "x = " << box_dim[0] << ";y = " << box_dim[1] << ";z = " << box_dim[2] << std::endl;
    std::cout << "Description des paramètres : " << config["data_args"]["description"] << std::endl;
    std::cout << "Résolution de l'octree : " << resolution <<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ref_cloud(new pcl::PointCloud<pcl::PointXYZI>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // The new cloud will also have a value for the labelling of each point
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::IndicesPtr IdxVector(new std::vector<int>());
    int count = 0; // Number of pointclouds in directory 
    fs::path extension(config["data_args"]["extension_type"]);

    /*-----------------------------------Get the files--------------------------------------------------*/
    // First we count the number of files, before storing them inside a vector
    for (const auto & entry : fs::directory_iterator(folder_path)){
        count ++;
    }
    vector<fs::path> txt_pointclouds(count); // Initialize the vector with good size
    int k = 0; // Iterator
    for (const auto & entry : fs::directory_iterator(folder_path)){
        txt_pointclouds[k] = entry.path().filename(); // Store the files name inside vector
        // cout << txt_pointclouds[k] << endl;
        k ++;
    }
    // Sorting the vector so that the files are ordered according to their timestamps
    sort(txt_pointclouds.begin(), txt_pointclouds.end());

    /*--------------------------------------PointCloud processing--------------------------------------*/
    // Creating the reference point cloud (without smoke)
    ref_cloud = referencePointCloud(ts1, ts2, folder_path, txt_pointclouds);
    std::cout << "SIZE OF REF_CLOUD : " << ref_cloud->size() << endl;

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    for (int i = config["data_args"]["last_ref_frame"]; i<txt_pointclouds.size(); i++){
        // Mesure time of each extraction
        auto t1 = high_resolution_clock::now();
        // Create Voxel grid
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree (resolution);
        octree.setInputCloud(ref_cloud);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();
        std::cout << "================================================================" <<  endl;
        // std::cout << "Processing frame " << i << "/" << txt_pointclouds.size() <<  endl;
        new_cloud = loadPointCloud(folder_path / txt_pointclouds[i]);
        // std::cout << "SIZE OF NEW_CLOUD : " << new_cloud->size() << endl;

        octree.setInputCloud(new_cloud); // Add points from new PointCloud
        octree.addPointsFromInputCloud(); 

        std::vector<int> newPointIdxVector;
        // Get vector of point indices from octree voxels which did not exist in previous buffer
        // std::cout << "Getting Indices" <<  endl;
        octree.getPointIndicesFromNewVoxels(newPointIdxVector);
        IdxVector.reset(new std::vector<int>());
        (*IdxVector).swap(newPointIdxVector);
        // std::cout << "SIZE OF INDEXES : " << IdxVector->size() << endl;
        // std::cout << "Indices stored, now extracting points" <<  endl;
        // Store the extarcted points and remove them from whole cloud
        extracted_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        extracted_cloud->height = 1;
        extracted_cloud->is_dense = true;
        for (int j=0; j<IdxVector->size(); j++){
            // std::cout << "current IdxVector : " << *IdxVector[j] << endl;
            (*extracted_cloud).push_back((*new_cloud)[(*IdxVector)[j]]);
        }
        // std::cout << "SIZE OF EXTRACTED CLOUD : " << extracted_cloud->size() << endl;
        // Erase the extracted points
        pcl::ExtractIndices<pcl::PointXYZI> eifilter(true); // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud(new_cloud);
        eifilter.setIndices(IdxVector);
        eifilter.setNegative(true);
        eifilter.filterDirectly(new_cloud);

        if (extracted_cloud->size() != 0){
            // Erase outlier points
            filtered_extracted_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;

            // RCLCPP_INFO(this->get_logger(), "Size of extracted cloud : %i", extracted_cloud->points.size());
            sor.setInputCloud(extracted_cloud);
            sor.setMeanK (40);
            sor.setStddevMulThresh (1.0);
            sor.filter (*filtered_extracted_cloud);
            // std::cout << "SIZE OF FILTERED EXTRACTED CLOUD : " << filtered_extracted_cloud->size() << endl;

            // Extract only the points around the position of the smoke
            cropped_extracted_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::CropBox<pcl::PointXYZI> box;
            Eigen::Vector4f minPoint = min_box(box_center, box_dim);
            Eigen::Vector4f maxPoint = max_box(box_center, box_dim);
            box.setMax(maxPoint);
            box.setMin(minPoint);
            box.setInputCloud(filtered_extracted_cloud);
            box.setNegative(false);
            box.filter((*cropped_extracted_cloud));
            // std::cout << "SIZE OF CROPPED EXTRACTED CLOUD : " << cropped_extracted_cloud->size() << endl;

            // Label the filtered extracted points and re add them in the whole point cloud
            PointCloudXYZIL::Ptr labelized_extracted_cloud(new PointCloudXYZIL());
            PointCloudXYZIL::Ptr labelized_cloud(new PointCloudXYZIL());

            pcl::copyPointCloud(*cropped_extracted_cloud, *labelized_extracted_cloud);
            pcl::copyPointCloud(*new_cloud, *labelized_cloud);
            // std::cout << "SIZE OF LABELIZED_CLOUD : " << labelized_cloud->size() << endl;
            for (int k=0; k<cropped_extracted_cloud->size(); k++){
                (*labelized_extracted_cloud)[k].label = SMOKE_LABEL;
                (*labelized_cloud).push_back((*labelized_extracted_cloud)[k]);
                // std::cout << "LABEL of last point" << (*labelized_cloud).back().label << std::endl;
            }
            // std::cout << "SIZE OF LABELIZED_CLOUD : " << labelized_cloud->size() << std::endl;

            auto t2 = high_resolution_clock::now();
            duration<double, std::milli> ms_double = t2 - t1;
            // std::cout << "Time before saving the frame : " << ms_double.count() << "ms" << std::endl;
            /*-------------------------------------Saving Clouds---------------------------------------------*/
            std::cout << "Saving Labelized pointclouds : "<< txt_pointclouds[i].replace_extension(extension) << endl;
            // pcl::io::savePCDFileBinaryCompressed (saving_path/txt_pointclouds[i].replace_extension(extension), *labelized_cloud);
            pcl::io::savePCDFileASCII (saving_path/txt_pointclouds[i].replace_extension(extension), *labelized_cloud);
            // pcl::io::savePCDFileBinaryCompressed (saving_path_extracted/txt_pointclouds[i].replace_extension(extension), *cropped_extracted_cloud);
            pcl::io::savePCDFileASCII (saving_path_extracted/txt_pointclouds[i].replace_extension(extension), *cropped_extracted_cloud);
            auto t3 = high_resolution_clock::now();
            duration<double, std::milli> ms_double2 = t3 - t1;
            // std::cout << "Time after saving the frames : " << ms_double2.count() << "ms" << std::endl;
            octree.deleteTree();
            labelized_cloud.reset();
            labelized_extracted_cloud.reset();
            new_cloud.reset();
            cout<<"labelized_cloud saved"<<endl;
        }
        else {
            std::cerr << "No points to filter!" << std::endl;
        }
        
    }
    
    return 0;
}