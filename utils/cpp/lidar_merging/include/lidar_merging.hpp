#ifndef LIDAR_MERGING_HPP
#define LIDAR_MERGING_HPP

#include "structures.hpp"
#include <iostream>
#include <fstream> 
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <math.h>       /* atan2 */
#include <filesystem>
#include <algorithm>    // std::sort
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <nlohmann/json.hpp>


/*--------------------PointXYZII------------------------*/
int LIDAR_DIMENSION = 4;
int NO_ID = 0;

struct EIGEN_ALIGN16 PointXYZII 
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;                  // add intensity  
    std::uint16_t id;              // int corresponding to the lidar id
    // Constructeur par défaut
    PointXYZII() : x(0), y(0), z(0), intensity(0), id(NO_ID){}

    // Constructeur de copie
    PointXYZII(const PointXYZII &other) = default;

    // Opérateur d'affectation
    PointXYZII& operator=(const PointXYZII &other) = default;

    // Opérateur de comparaison (pour les recherches)
    bool operator==(const PointXYZII &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    std::ostream& operator<<(ostream& os) const{
        os << "x = " << x << ";y = " << x << ";z = " << x << ";intensity = " << intensity << ";ID = " << id;  
        return os;
    }

    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};                      // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZII,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint16_t, id, id)
)

typedef pcl::PointCloud<PointXYZII> PointCloudXYZII;
// // Overload the + operator to concatenate two PointCloudXYZII
// PointCloudXYZII operator+(const PointCloudXYZII& cloud1, const PointCloudXYZII& cloud2) {
//     PointCloudXYZII result;
//     result.reserve(cloud1.size() + cloud2.size());

//     // Add points from the first cloud
//     result.insert(result.end(), cloud1.begin(), cloud1.end());

//     // Add points from the second cloud
//     result.insert(result.end(), cloud2.begin(), cloud2.end());

//     return result;
// }

// // Overload the += operator to append points from another cloud
// PointCloudXYZII& operator+=(PointCloudXYZII& cloud1, const PointCloudXYZII& cloud2) {
//     // Append points from cloud2 to cloud1
//     cloud1.insert(cloud1.end(), cloud2.begin(), cloud2.end());
//     return cloud1;
// }

void CopyPointXYZIToXYZII(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, pcl::PointCloud<PointXYZII>::Ptr cloudOut, int id){
    // Resize the output cloud
    cloudOut->resize(cloudIn->size());
    // Copy and convert
    for (size_t i = 0; i < cloudIn->size(); ++i) {
        cloudOut->points[i].x = cloudIn->points[i].x;
        cloudOut->points[i].y = cloudIn->points[i].y;
        cloudOut->points[i].z = cloudIn->points[i].z;
        cloudOut->points[i].intensity = cloudIn->points[i].intensity;
        cloudOut->points[i].id = id;
    }
}

/*--------------------PointMoCap------------------------*/


struct PointMoCap 
{
    pcl::PointXYZ point;           // XYZ pclPoint
    Quaternion quat;               // add Quaternion for orientation of frame  
    std::uint16_t id;              // int corresponding to the lidar id
    // Constructeur par défaut
    PointMoCap() : point(), quat(), id(NO_ID){}

    // Constructeur avec point, quaternion et id  
    PointMoCap(const pcl::PointXYZ point_, Quaternion quat_, int id_) : point(point_), quat(quat_), id(id_){}

    // Constructeur de copie
    PointMoCap(const PointMoCap &other) = default;

    // Conversion en transformation affine Eigen
    Eigen::Affine3d toAffine3d() const {
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        // Appliquer la translation à partir du point
        transform.translate(Eigen::Vector3d(point.x, point.y, point.z));
        // Appliquer la rotation à partir du quaternion
        transform.rotate(quat.toRotationMatrix());
        return transform;
    }

    // Mise à jour depuis une transformation affine Eigen
    void fromAffine3d(const Eigen::Affine3d& transform) {
        // Extraire la translation
        point.x = transform.translation().x();
        point.y = transform.translation().y();
        point.z = transform.translation().z();

        // Extraire la rotation (quaternion)
        Eigen::Matrix3d rotationMatrix = transform.linear();
        Eigen::Quaterniond eigenQuat(rotationMatrix);
        quat = Quaternion(eigenQuat.w(), eigenQuat.x(), eigenQuat.y(), eigenQuat.z());
    }

    // Opérateur d'affectation
    PointMoCap& operator=(const PointMoCap &other) = default;

    // Opérateur de comparaison (pour les recherches)
    bool operator==(const PointMoCap &other) const
    {
        return point.x==other.point.x && point.y==other.point.y && point.z==other.point.z && quat == other.quat;
    }

    void print(){
        std::cout << "x = " << point.x << ";y = " << point.y << ";z = " << point.z << ";quaternion (qw, qx, qy, qz) = " << quat << ";ID = " << id << std::endl;  
    }

    using Ptr = std::shared_ptr<PointMoCap>;
    using ConstPtr = std::shared_ptr<const PointMoCap>;

};                      // enforce SSE padding for correct memory alignment

void printAffine3d(const Eigen::Affine3d& transform, const std::string& name = "Transformation") {
    std::cout << name << " matrix (4x4):" << std::endl;
    std::cout << transform.matrix() << std::endl;
}


#endif // LIDAR_MERGING_HPP