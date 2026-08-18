#ifndef SMOKE_LABELLING_HPP
#define SMOKE_LABELLING_HPP

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
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <nlohmann/json.hpp>

int LIDAR_DIMENSION = 4;
int NOT_LABELIZED = 0;
int SMOKE_LABEL = 1;

struct EIGEN_ALIGN16 PointXYZIL 
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;                  // add intensity  
    std::uint16_t label;              // int corresponding to a label
    // Constructeur par défaut
    PointXYZIL() : x(0), y(0), z(0), intensity(0), label(NOT_LABELIZED){}

    // Constructeur de copie
    PointXYZIL(const PointXYZIL &other) = default;

    // Opérateur d'affectation
    PointXYZIL& operator=(const PointXYZIL &other) = default;

    // Opérateur de comparaison (pour les recherches)
    bool operator==(const PointXYZIL &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    std::ostream& operator<<(ostream& os) const{
        os << "x = " << x << ";y = " << x << ";z = " << x << ";intensity = " << intensity << ";LABEL = " << label;  
        return os;
    }

    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};                      // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint16_t, label, label)
)

typedef pcl::PointCloud<PointXYZIL> PointCloudXYZIL;


#endif // SMOKE_LABELLING_HPP