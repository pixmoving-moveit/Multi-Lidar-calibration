#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <QObject>
#include <QThread>
#include <iostream>
#include <string>

#include <iostream>
#include <vector>
#include <omp.h>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <regex>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include"check.h"

#include "include/json.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
using json = nlohmann::json;

enum QOS_Level {
    SYSTEM_DEFAULT = 0,
    BEST_EFFORT = 1,
    RELIABLE = 2
};

struct SensorIndex
{
    int num;
    std::string sensor_frame;
    std::string calibration_file_path;
};

struct InitParams
{
    SensorIndex sensor_index;
    LidarConfig lidar_front_top;
    LidarConfig lidar_front_right;
    LidarConfig lidar_front_left;
    LidarConfig lidar_rear_top;
    LidarConfig lidar_rear_center;
    
    QOS_Level qos;
};


// 消息处理任务结构
struct PointCloudTask
{
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    std::string lidar_type;  // "front_top", "front_right", "front_left", "rear_top", "rear_center"
    
    PointCloudTask(sensor_msgs::msg::PointCloud2::SharedPtr msg_, const std::string& type_)
        : msg(msg_), lidar_type(type_) {}
};



class rclcomm:public QThread
{
    Q_OBJECT
public:

    InitParams init_params;
    Check* check_instance;
    bool switch_ground_wall;

    static rclcomm* getInstance();
    
    rclcomm();
    ~rclcomm();
        bool readParamsFromJson(const std::string& file_path, InitParams& params);
        bool readCalibrationFromYaml(const std::string& file_path, InitParams& params);
        bool writeYamlFromParams(const std::string& file_path, const InitParams& params);
        void setGroundCheckStatusByName(const std::string& lidar_name, const bool& is_checked);
        void setStereoCheckStatusByName(const std::string& lidar_name, const bool& is_checked);
        void setAxisModifyByName(const std::string& lidar_name, const std::vector<double>& values);
        void setRotateModifyByName(const std::string& lidar_name, const std::vector<double>& values);

    protected:
        void run();
    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_front_top_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_front_right_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_front_left_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_rear_top_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_rear_center_subscriber_;

        std::shared_ptr<rclcpp::Node> node;
        std::vector<std::vector<BoxInfo>> boxes_show_in_rviz;
    
    // 静态tf广播器
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

private:

    void CallbackLidar_front_top(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud);
    void CallbackLidar_front_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud);
    void CallbackLidar_front_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud);
    void CallbackLidar_rear_top(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud);
    void CallbackLidar_rear_center(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud);

    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher, const std::string& frame_id);
    void publishBoxes(const std::vector<std::vector<BoxInfo>>& boxes, const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher, const std::string& frame_id);
    void publishStaticTfTransforms(); // 发布静态tf变换关系

    void transformPointcloudWithTF2(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& config);
    void transformPointcloudWithParams(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& config);
    void extractPlatesPointsByIntens(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const PlateParams& plate_params);
    void extractGroundPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& lidar_config);
    void extractWallPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& lidar_config);
    void extractBoxesPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const std::vector<pcl::PointXYZI>& boxes_info);
    int clusterFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_output, const float& dis = 0.4);
    BoxInfo obbBox(const pcl::PointCloud<pcl::PointXYZI>& cloud_input);
    int obbBoxFilter(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_output, std::vector<BoxInfo>& boxs_info, const PlateParams& plate_params);
    void phraseRegionGround(const json& region_ground, LidarConfig& lidar_config);
    void phraseRegionWall(const json& json_, const std::string& region_wall_label, LidarConfig& lidar_config);
    void phraseStereoPlate(const json& intensity_plates, LidarConfig& lidar_config);
    void extractYAxisWallPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, const std::vector<pcl::PointXYZI>& boxes_center, std::vector<PlateInfo>& plates, std::vector<BoxInfo>& boxes);

private:
    // 线程池相关成员变量
    std::vector<std::thread> worker_threads_;
    std::queue<PointCloudTask> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::atomic<bool> shutdown_flag_;
    static const size_t THREAD_POOL_SIZE = 5;  // 线程池大小

    // 线程处理相关方法
    void initializeThreadPool();
    void shutdownThreadPool();
    void workerThread();
    void processPointCloudTask(const PointCloudTask& task);
    void processLidarFrontTop(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void processLidarFrontRight(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void processLidarFrontLeft(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void processLidarRearTop(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void processLidarRearCenter(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

signals:
    void emitShowData();
};
#endif // RCLCOMM_H
