#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tuple>
#include <list>
#include <pcl/filters/voxel_grid.h>


#include <chrono>

#define DEFAULT_DECAY_TIME 1
#define DEFAULT_PCL_FREQ 400
#define DEFAULT_FRAME_ID "world"


template <typename T>
class Buffer{
    public:
        Buffer(){};
        Buffer(uint max_size){
            _max_size = max_size;
        };

        virtual void push(T element){
            _buffer.push_back(element);
            if (_buffer.size() > _max_size){
                _buffer.pop_front();
            }
        };

        int max_size(){
            return _max_size;
        }

        int size(){
            return _buffer.size();
        }

        void set_max_size(int val){
            _max_size = val;
        }

    protected:
        std::list<T> _buffer;
    private:
        uint _max_size;
};

template<typename T>
class PCLBuffer: public Buffer<std::shared_ptr<pcl::PointCloud<T>>>{
    public:
        PCLBuffer(){};
        PCLBuffer(double max_time, double dt): Buffer<std::shared_ptr<pcl::PointCloud<T>>>(max_time/dt){
        };
    
        void push(std::shared_ptr<pcl::PointCloud<T>> cloud) override{  

            // process pcl element first to clean it up
            
            pcl::VoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud (cloud);
            sor.setLeafSize (leaf_size[0], leaf_size[1], leaf_size[2]);
            sor.filter (*cloud);

            Buffer<std::shared_ptr<pcl::PointCloud<T>>>::push(cloud);
        };

        void create_mesh(std::shared_ptr<pcl::PointCloud<T>> mesh){
            for (auto cloud: this->_buffer){
                *mesh += *cloud;
            }
        }

    private:
        std::array<float, 3> leaf_size = {0.0002f, 0.0002f, 0.0002f};
};
