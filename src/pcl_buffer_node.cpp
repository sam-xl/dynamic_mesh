#include <dynamic_mesh/pcl_buffer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>

template <typename PointType>
class PCLBufferNode : public rclcpp::Node{
    public:
        PCLBufferNode():rclcpp::Node("pcl_buffer_node"), 
                        pcl_buffer(DEFAULT_PCL_FREQ, DEFAULT_PCL_FREQ),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
        {
            this->declare_parameter<int>("decay_time", DEFAULT_DECAY_TIME);
            this->get_parameter_or("decay_time", _decay_time, DEFAULT_DECAY_TIME);

            this->declare_parameter<int>("pcl_frequency", DEFAULT_PCL_FREQ);
            this->get_parameter_or("pcl_frequency", _pcl_frequency, DEFAULT_PCL_FREQ);

            this->declare_parameter<std::string>("frame_id", std::string(DEFAULT_FRAME_ID));
            this->get_parameter_or("frame_id", _frame_id, std::string(DEFAULT_FRAME_ID));

            this->declare_parameter<bool>("do_transform", false);
            this->get_parameter_or("do_transform", _do_transform, false);

            pcl_buffer.set_max_size(_decay_time*_pcl_frequency); // reset it here because we need intializer lists for template classes

            pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("input_cloud", 10, std::bind(&PCLBufferNode::pcl_callback, this, std::placeholders::_1));
            pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_mesh", 10);

            RCLCPP_INFO(this->get_logger(), "Started pcl buffer node.");
        }

        void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {   
            // Transform the point cloud
            if (_do_transform){
                try{
                    geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_.lookupTransform(_frame_id, msg->header.frame_id, tf2::TimePointZero);
                }
                catch(const tf2::LookupException& e){
                    std::cerr <<"Lookup transform failed: "<< e.what() <<" Trying again.."<< '\n';
                }
                
                tf2::doTransform(*msg, *msg, transform_stamped);
                msg->header.frame_id = _frame_id;
                
            }

            typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            pcl::fromROSMsg(*msg, *cloud);

            pcl_buffer.push(cloud);
                            
            sensor_msgs::msg::PointCloud2 pub_msg;
            typename pcl::PointCloud<PointType>::Ptr mesh(new pcl::PointCloud<PointType>);
            pcl_buffer.create_mesh(mesh);

            pcl::toROSMsg(*mesh, pub_msg);
            pub_msg.header.frame_id = _frame_id;
            pcl_pub->publish(pub_msg);
        }


    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
        PCLBuffer<PointType> pcl_buffer;
        
        // TF2 Buffer and listener
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;


        // parameters
        int _decay_time;
        int _pcl_frequency;
        std::string _frame_id;
        bool _do_transform; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PCLBufferNode<pcl::PointXYZI>>());
    rclcpp::shutdown();

    return 0;
}
