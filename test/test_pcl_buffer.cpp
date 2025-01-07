#include <defect_detection/pcl_buffer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

template <typename PointType>
class PCLBufferNode : public rclcpp::Node
{
public:
        PCLBufferNode(double max_time, double freq):rclcpp::Node("pcl_buffer_node"), buffer(max_time, 1/freq)
        {

            std::cout << buffer.max_size() << std::endl;
            pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/transformed_pointcloud", 10, std::bind(&PCLBufferNode::pcl_callback, this, std::placeholders::_1));
            pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_mesh", 10);

        }

        void pcl_callback(const sensor_msgs::msg::PointCloud2 &msg)
        {
            typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(msg, *cloud);

            buffer.push(cloud);
                            
            sensor_msgs::msg::PointCloud2 pub_msg;
            typename pcl::PointCloud<PointType>::Ptr mesh(new pcl::PointCloud<PointType>);
            buffer.create_mesh(mesh);

            pcl::toROSMsg(*mesh, pub_msg);
            pub_msg.header.frame_id = "world";
            pcl_pub->publish(pub_msg);
        }


    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
        PCLBuffer<PointType> buffer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLBufferNode<pcl::PointXYZI>>(1, 400));

    rclcpp::shutdown();

    return 0;
}