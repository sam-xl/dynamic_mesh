#include <dynamic_mesh/pcl_buffer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


int TEST_max_size()
{
    int time = 1;    // seconds
    double dt = 0.1; // seconds
    PCLBuffer<pcl::PointXYZI> buffer(time, dt);

    std::cout << buffer.max_size() << std::endl;

    for (int i = 0; i < 20; i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        buffer.push(cloud);
    }

    std::cout << buffer.size() << std::endl; // should be equal to max size

    return 0;
}


