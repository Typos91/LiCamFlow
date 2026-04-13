#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>

struct EIGEN_ALIGN16 MyPointType
{
    PCL_ADD_POINT4D;
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

int main()
{
    pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
    MyPointType p;
    p.x = 1.0; p.y = 2.0; p.z = 3.0; p.intensity = 100.0;
    cloud->push_back(p);

    float resolution = 128.0f;
    pcl::octree::OctreePointCloud<MyPointType> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    return 0;
}





