#include "rclcomm.h"
#include <filesystem>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


rclcomm::rclcomm()
{
    int argc=0;
    char **argv=NULL;
    rclcpp::init(argc,argv);
    node=rclcpp::Node::make_shared("lidar_calibration_verif");

    // 声明参数
    node->declare_parameter("config_file_path", "");
    node->declare_parameter("calibration_file_path", "");
    
    // 获取参数值
    std::string config_path = node->get_parameter("config_file_path").as_string();
    std::string calibration_path = node->get_parameter("calibration_file_path").as_string();
    
    // 如果参数为空，使用默认的相对路径
    if (config_path.empty()) {
        // 使用相对于包安装目录的路径
        auto package_path = ament_index_cpp::get_package_share_directory("lidar_calibration_verif");
        config_path = package_path + "/config/config.json";
    }
    
    if (calibration_path.empty()) {
        auto package_path = ament_index_cpp::get_package_share_directory("lidar_calibration_verif");
        calibration_path = package_path + "/config/sensor_kit_calibration.yaml";
    }
    
    if (this->readParamsFromJson(config_path, init_params))    std::cout << "Successfully loaded configuration from: " << config_path << std::endl;
    else  std::cerr << "Failed to load configuration from: " << config_path << std::endl;

    // 读取雷达标定数据
    if (this->readCalibrationFromYaml(calibration_path, init_params)) {
        std::cout << "Successfully loaded lidar calibration from: " << calibration_path << std::endl;
    } else {
        std::cerr << "Failed to load lidar calibration from: " << calibration_path << std::endl;
    }

    points_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points_front_right", 10); //测试用的点云发布器
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("calibration_boxes", 10);
    
    // 创建静态tf广播器
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    if (init_params.qos == QOS_Level::BEST_EFFORT) {
        qos_profile.best_effort();
    } else if (init_params.qos == QOS_Level::RELIABLE) {
        qos_profile.reliable();
    }
    // 默认QOS_Level::QOS_DEFAULT时不做额外设置

    // 分别订阅各个雷达的点云
    points_front_top_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        init_params.lidar_front_top.ros_topic,
        qos_profile,
        std::bind(&rclcomm::CallbackLidar_front_top, this, std::placeholders::_1));
    points_front_right_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(init_params.lidar_front_right.ros_topic,
            qos_profile,
            std::bind(&rclcomm::CallbackLidar_front_right, this, std::placeholders::_1));
    points_front_left_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(init_params.lidar_front_left.ros_topic,
            qos_profile,
            std::bind(&rclcomm::CallbackLidar_front_left, this, std::placeholders::_1));
    points_rear_top_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(init_params.lidar_rear_top.ros_topic,
            qos_profile,
            std::bind(&rclcomm::CallbackLidar_rear_top, this, std::placeholders::_1));
    points_rear_center_subscriber_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(init_params.lidar_rear_center.ros_topic,
            qos_profile,
            std::bind(&rclcomm::CallbackLidar_rear_center, this, std::placeholders::_1));


    check_instance = Check::getInstance();

    boxes_show_in_rviz.resize(init_params.sensor_index.num);

    switch_ground_wall = true; //默认打开时在地面校验界面

    // 初始化线程池
    initializeThreadPool();

    this->start();
}

rclcomm::~rclcomm()
{
    // 停止QThread的运行循环
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    
    // 等待QThread结束
    if (this->isRunning()) {
        this->quit();           // 请求线程退出
        this->wait(5000);       // 等待最多5秒
        if (this->isRunning()) {
            this->terminate();  // 强制终止线程
            this->wait(1000);   // 等待终止完成
        }
    }
    
    // 关闭线程池
    shutdownThreadPool();
    
    std::cout << "rclcomm destructor completed." << std::endl;
}

rclcomm* rclcomm::getInstance()
{
    static rclcomm instance;
    return &instance;
}

void rclcomm::run()
{
    //20HZ
    rclcpp::WallRate loop_rate(1);
    while (rclcpp::ok() && !this->isInterruptionRequested())
    {
        // 检查各雷达3秒内是否收到新消息，更新is_active标志
        rclcpp::Time now = node->now();
        auto check_active = [&](auto& lidar) {
            if (lidar.header.stamp.sec != 0 || lidar.header.stamp.nanosec != 0) 
            {
                rclcpp::Time last_msg_time(lidar.header.stamp);
                lidar.is_active = (now - last_msg_time).seconds() < 3.0;
            } else {
            lidar.is_active = false;
            }
        };
        check_active(init_params.lidar_front_top);
        check_active(init_params.lidar_front_right);
        check_active(init_params.lidar_front_left);
        check_active(init_params.lidar_rear_top);
        check_active(init_params.lidar_rear_center);

        emitShowData();

        // 发布静态tf变换关系
        publishStaticTfTransforms();

        publishBoxes(boxes_show_in_rviz, marker_pub_, init_params.sensor_index.sensor_frame);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    std::cout << "rclcomm thread exiting gracefully." << std::endl;
}

void rclcomm::CallbackLidar_front_top(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud)
{
    init_params.lidar_front_top.header.stamp = node->now();
    // 将任务添加到队列中，由工作线程处理
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.emplace(msg_point_cloud, "front_top");
    }
    queue_condition_.notify_one();
}

void rclcomm::CallbackLidar_front_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud)
{
    init_params.lidar_front_right.header.stamp = node->now();
    // 将任务添加到队列中，由工作线程处理
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.emplace(msg_point_cloud, "front_right");
    }
    queue_condition_.notify_one();
}

void rclcomm::CallbackLidar_front_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud)
{
    init_params.lidar_front_left.header.stamp = node->now();
    // 将任务添加到队列中，由工作线程处理
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.emplace(msg_point_cloud, "front_left");
    }
    queue_condition_.notify_one();
}

void rclcomm::CallbackLidar_rear_top(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud)
{
    init_params.lidar_rear_top.header.stamp = node->now();
    // 将任务添加到队列中，由工作线程处理
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.emplace(msg_point_cloud, "rear_top");
    }
    queue_condition_.notify_one();
}

void rclcomm::CallbackLidar_rear_center(const sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud)
{
    init_params.lidar_rear_center.header.stamp = node->now();
    // 将任务添加到队列中，由工作线程处理
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        task_queue_.emplace(msg_point_cloud, "rear_center");
    }
    queue_condition_.notify_one();
}

void rclcomm::extractPlatesPointsByIntens(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const PlateParams& plate_params)
{
    cloud_output.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_output->points.reserve(cloud_input->points.size());

    float min_intensity = plate_params.min_intensity;
    float max_intensity = plate_params.max_intensity;

    omp_set_num_threads(8); //设置多线程数量

    #pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZI> local_cloud;
        local_cloud.points.reserve(cloud_input->points.size() / omp_get_num_threads() + 1);

        #pragma omp for nowait
        for (size_t i = 0; i < cloud_input->points.size(); ++i) {
            const auto& pt = cloud_input->points[i];
            if (pt.x < 4.0 &&pt.intensity >= min_intensity && pt.intensity <= max_intensity) {
                local_cloud.points.push_back(pt);
            }
        }

        #pragma omp critical
        {
            cloud_output->points.insert(cloud_output->points.end(), local_cloud.points.begin(), local_cloud.points.end());
        }
    }

    cloud_output->width = static_cast<uint32_t>(cloud_output->points.size());
    cloud_output->height = 1;
    cloud_output->is_dense = true;
}

void rclcomm::extractGroundPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& lidar_config)
{
    cloud_output->clear();
    
    if (cloud_input->empty() || lidar_config.region_ground.empty()) {
        return;
    }
    
    // 预分配输出点云的大概大小
    cloud_output->reserve(cloud_input->size());
    
    // 使用临时向量存储筛选结果的索引，避免多线程写入冲突
    std::vector<bool> point_in_region(cloud_input->size(), false);
    
    // 使用OpenMP并行化处理
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < cloud_input->size(); ++i) {
        const auto& point = cloud_input->points[i];
        
        // 检查点是否在任一矩形区域内
        for (const auto& region : lidar_config.region_ground) {
            float center_x = region.x;  // 矩形中心x坐标
            float center_y = region.y;  // 矩形中心y坐标
            float half_size = region.intensity / 2.0f;  // 矩形半边长
            
            // 检查点是否在当前矩形区域内
            if (point.x >= (center_x - half_size) && point.x <= (center_x + half_size) &&
                point.y >= (center_y - half_size) && point.y <= (center_y + half_size)) {
                point_in_region[i] = true;
                break;  // 找到一个匹配区域即可退出内层循环
            }
        }
    }
    
    // 串行收集结果点
    for (size_t i = 0; i < cloud_input->size(); ++i) {
        if (point_in_region[i]) {
            cloud_output->push_back(cloud_input->points[i]);
        }
    }
    
    // 设置输出点云属性
    cloud_output->width = cloud_output->size();
    cloud_output->height = 1;
    cloud_output->is_dense = true;
    cloud_output->header = cloud_input->header;
}

void rclcomm::extractBoxesPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const std::vector<pcl::PointXYZI>& boxes_info)
{
    cloud_output->clear();

    if (cloud_input->empty() || boxes_info.empty()) {
        return;
    }

    for (size_t i = 0; i < boxes_info.size(); i++)
    {
        pcl::PointXYZI box = boxes_info[i];
        float half_size = box.intensity / 2.0f; // intensity存储size
        // 定义立方体中心点和边长
        Eigen::Vector4f min_pt(box.x - half_size, box.y - half_size, box.z - half_size, 1.0f);  // 立方体最小坐标
        Eigen::Vector4f max_pt(box.x + half_size, box.y + half_size, box.z + half_size, 1.0f);     // 立方体最大坐标

        // 创建 CropBox 过滤器
        pcl::CropBox<pcl::PointXYZI> cropBoxFilter;
        cropBoxFilter.setInputCloud(cloud_input);
        cropBoxFilter.setMin(min_pt);
        cropBoxFilter.setMax(max_pt);

        // 提取立方体内的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        cropBoxFilter.filter(*cloud_filtered);
        cloud_output->points.insert(cloud_output->points.end(), cloud_filtered->points.begin(), cloud_filtered->points.end());
    }
    
    // 设置输出点云属性
    cloud_output->width = cloud_output->size();
    cloud_output->height = 1;
    cloud_output->is_dense = true;
    cloud_output->header = cloud_input->header;
}

void rclcomm::extractWallPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& lidar_config)
{
    cloud_output->clear();
    
    if (cloud_input->empty() || lidar_config.region_wall_x.empty()) {
        return;
    }
    
    // 预分配输出点云的大概大小
    cloud_output->reserve(cloud_input->size());
    
    // 使用临时向量存储筛选结果的索引，避免多线程写入冲突
    std::vector<bool> point_in_region(cloud_input->size(), false);
    
    // 使用OpenMP并行化处理
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < cloud_input->size(); ++i) {
        const auto& point = cloud_input->points[i];
        
        // 检查点是否在任一矩形区域内
        for (const auto& region : lidar_config.region_wall_x) {
            float center_y = region.y;  // 矩形中心y坐标
            float center_z = region.z;  // 矩形中心z坐标
            float half_size = region.intensity / 2.0f;  // 矩形半边长
            
            // 检查点是否在当前矩形区域内
            if (point.y >= (center_y - half_size) && point.y <= (center_y + half_size) &&
                point.z >= (center_z - half_size) && point.z <= (center_z + half_size) &&
                fabs(point.x) > 3.0f) { // 限制x坐标在3.0范围外，用于剔除车身点云影响
                point_in_region[i] = true;
                break;  // 找到一个匹配区域即可退出内层循环
            }
        }
    }
    
    // 串行收集结果点
    for (size_t i = 0; i < cloud_input->size(); ++i) {
        if (point_in_region[i]) {
            cloud_output->push_back(cloud_input->points[i]);
        }
    }
    
    // 设置输出点云属性
    cloud_output->width = cloud_output->size();
    cloud_output->height = 1;
    cloud_output->is_dense = true;
    cloud_output->header = cloud_input->header;
}


void rclcomm::transformPointcloudWithTF2(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& config)
{
    // 查询TF关系并将点云从子坐标系(config.frame_id)转换到父坐标系(params.sensor_index.sensor_frame)
    cloud_output.reset(new pcl::PointCloud<pcl::PointXYZI>);
    try {
        // 创建tf2缓冲区和监听器
        static tf2_ros::Buffer tf_buffer(node->get_clock());
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        geometry_msgs::msg::TransformStamped transform_stamped;
        // 查询变换
        transform_stamped = tf_buffer.lookupTransform(
            init_params.sensor_index.sensor_frame, // 父坐标系
            config.frame_id,                       // 子坐标系
            rclcpp::Time(0),                       // 最新时间
            rclcpp::Duration::from_seconds(0.1)    // 超时时间
        );

        // 转换为Eigen矩阵
        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);
        Eigen::Matrix4d transform_matrix = transform_eigen.matrix().cast<double>();

        // 点云变换
        pcl::transformPointCloud(*cloud_input, *cloud_output, transform_matrix);

    } catch (const tf2::TransformException& ex) {
        std::cerr << "Could not transform pointcloud from " << config.frame_id << " to " << init_params.sensor_index.sensor_frame << ": " << ex.what() << std::endl;
        cloud_output = cloud_input; // 转换失败则直接输出原始点云
    }
}

void rclcomm::transformPointcloudWithParams(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_output, const LidarConfig& config)
{
    // 使用预加载的标定变换矩阵进行点云变换
    cloud_output.reset(new pcl::PointCloud<pcl::PointXYZI>);
    
    try {
        // 直接使用预计算的变换矩阵
        auto transform_matrix = config.getTransMatrix(); // 获取雷达到sensor_kit_base_link的变换矩阵
        pcl::transformPointCloud(*cloud_input, *cloud_output, transform_matrix);

        // std::cout << "Transformed pointcloud using Params matrix for " << config.sensor_name
        //           << " with " << cloud_input->size() << " points." << std::endl;
                  
    } catch (const std::exception& e) {
        std::cerr << "Error transforming pointcloud with Params matrix: " << e.what() << std::endl;
        cloud_output = cloud_input; // 转换失败则直接输出原始点云
    }
}

int rclcomm::clusterFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_output, const float& dis)
{
    if(cloud_input->size() < 5) return 0;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(dis); // 距离阈值，可根据需要调整
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_input);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("clusterFilter"), "No clusters found in PointsCloud.");
        return false;
    }

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI> cluster;
        for (const auto& idx : indices.indices) {
            cluster.points.push_back(cloud_input->points[idx]);
        }
        cluster.width = static_cast<uint32_t>(cluster.points.size());
        cluster.height = 1;
        cluster.is_dense = true;
        cloud_output.push_back(cluster);
    }
    return static_cast<int>(cloud_output.size());
}

/**
 * 计算点云簇的三维尺寸，期望得到符合标定板特征的点云簇
 */
BoxInfo rclcomm::obbBox(const pcl::PointCloud<pcl::PointXYZI>& cloud_input)
{
    if (cloud_input.empty()) {
        std::cout << "Input cloud is empty, cannot compute OBB." << std::endl;
        return BoxInfo(); // 返回一个空的BoxInfo对象
    }

    // 1. 计算主方向
    pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cloud_input.makeShared());
    feature_extractor.compute();

    pcl::PointXYZI min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    Eigen::Vector3f obb_size = max_point_OBB.getVector3fMap() - min_point_OBB.getVector3fMap();
    float length = obb_size.x();
    float width  = obb_size.y();
    float height = obb_size.z();

    // 规定xyz中最小值为高度
    float dims[3] = {length, width, height};
    std::sort(dims, dims + 3);
    // 输出OBB的尺寸
    float obb_height = dims[0];
    float obb_width  = dims[1];
    float obb_length = dims[2];

    // std::cout << "OBB size (LxWxH): " << obb_length << " x " << obb_width << " x " << obb_height << std::endl;

    BoxInfo box_info;
    box_info.size = Eigen::Vector3f(obb_length, obb_width, obb_height);
    box_info.position = position_OBB.getVector3fMap();
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    box_info.orientation = quat;

    return box_info;

    
}

int rclcomm::obbBoxFilter(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_input, std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_output, std::vector<BoxInfo>& boxes_info, const PlateParams& plate_params)
{
    if (cloud_input.empty()) {
        // std::cout << "Input cloud vector is empty, cannot compute OBB." << std::endl;
        return 0;
    }

    for (const auto& cloud : cloud_input) {
        BoxInfo obb_size = obbBox(cloud);
        
        //这里的条件需要修改
        if (obb_size.size.x() > plate_params.size_min_x && obb_size.size.y() > plate_params.size_min_y && obb_size.size.z() < plate_params.size_max_z)
        {
            cloud_output.push_back(cloud);
            boxes_info.push_back(obb_size);
        }
    }

    // std::cout << "Filtered " << cloud_output.size() << " clouds based on OBB size." << std::endl;
    return static_cast<int>(cloud_output.size());
}


void rclcomm::publishBoxes(
    const std::vector<std::vector<BoxInfo>>& boxes,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const std::string& frame_id)
{
    if (!publisher) return;
    std::vector<BoxInfo> boxList;
    // 将boxes中的所有元素放置到boxList中
    for (const auto& box_vec : boxes) {
        boxList.insert(boxList.end(), box_vec.begin(), box_vec.end());
    }

    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < boxList.size(); ++i) {
        const auto& box = boxList[i];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node->now();
        marker.ns = "boxes";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = box.position.x();
        marker.pose.position.y = box.position.y();
        marker.pose.position.z = box.position.z();
        marker.pose.orientation.x = box.orientation.x();
        marker.pose.orientation.y = box.orientation.y();
        marker.pose.orientation.z = box.orientation.z();
        marker.pose.orientation.w = box.orientation.w();
        marker.scale.x = box.size.x();
        marker.scale.y = box.size.y();
        marker.scale.z = box.size.z();
        if (box.color.a > 0.0f) {
            marker.color = box.color;
        } else {
            marker.color.r = box.color.r;
            marker.color.g = box.color.g;
            marker.color.b = box.color.b;
            marker.color.a = box.color.a;
        }
        marker.lifetime = rclcpp::Duration::from_seconds(2.0); // 设置marker的生命周期 2s
        marker_array.markers.push_back(marker);
    }
    publisher->publish(marker_array);
}

void rclcomm::publishPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
    const std::string& frame_id)
{
    if (!cloud || !publisher) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = node->now();
    msg.header.frame_id = frame_id;
    publisher->publish(msg);
}

void rclcomm::setGroundCheckStatusByName(const std::string& lidar_name, const bool& is_checked)
{
    if (lidar_name == init_params.lidar_front_top.sensor_name) {
        init_params.lidar_front_top.is_ground_checked = is_checked;
    } else if (lidar_name == init_params.lidar_front_right.sensor_name) {
        init_params.lidar_front_right.is_ground_checked = is_checked;
    } else if (lidar_name == init_params.lidar_front_left.sensor_name) {
        init_params.lidar_front_left.is_ground_checked = is_checked;
    } else if (lidar_name == init_params.lidar_rear_top.sensor_name) {
        init_params.lidar_rear_top.is_ground_checked = is_checked;
    } else if (lidar_name == init_params.lidar_rear_center.sensor_name) {
        init_params.lidar_rear_center.is_ground_checked = is_checked;
    }
}

void rclcomm::setStereoCheckStatusByName(const std::string& lidar_name, const bool& is_checked)
{
    if (lidar_name == init_params.lidar_front_top.sensor_name) {
        init_params.lidar_front_top.is_stereo_checked = is_checked;
    } else if (lidar_name == init_params.lidar_front_right.sensor_name) {
        init_params.lidar_front_right.is_stereo_checked = is_checked;
    } else if (lidar_name == init_params.lidar_front_left.sensor_name) {
        init_params.lidar_front_left.is_stereo_checked = is_checked;
    } else if (lidar_name == init_params.lidar_rear_top.sensor_name) {
        init_params.lidar_rear_top.is_stereo_checked = is_checked;
    } else if (lidar_name == init_params.lidar_rear_center.sensor_name) {
        init_params.lidar_rear_center.is_stereo_checked = is_checked;
    }
}

void rclcomm::setAxisModifyByName(const std::string& lidar_name, const std::vector<double>& values)
{
    if (values.size() != 3) return; // 确保传入的values大小正确

    if (lidar_name == init_params.lidar_front_top.sensor_name) {
        if(fabs(values[0]) > 1e-3) init_params.lidar_front_top.transform.x += values[0];
        if(fabs(values[1]) > 1e-3) init_params.lidar_front_top.transform.y += values[1];
        if(fabs(values[2]) > 1e-3) init_params.lidar_front_top.transform.z += values[2];
    } else if (lidar_name == init_params.lidar_front_right.sensor_name) {
        if(fabs(values[0]) > 1e-3) init_params.lidar_front_right.transform.x += values[0];
        if(fabs(values[1]) > 1e-3) init_params.lidar_front_right.transform.y += values[1];
        if(fabs(values[2]) > 1e-3) init_params.lidar_front_right.transform.z += values[2];
    } else if (lidar_name == init_params.lidar_front_left.sensor_name) {
        if(fabs(values[0]) > 1e-3) init_params.lidar_front_left.transform.x += values[0];
        if(fabs(values[1]) > 1e-3) init_params.lidar_front_left.transform.y += values[1];
        if(fabs(values[2]) > 1e-3) init_params.lidar_front_left.transform.z += values[2];
    } else if (lidar_name == init_params.lidar_rear_top.sensor_name) {
        if(fabs(values[0]) > 1e-3) init_params.lidar_rear_top.transform.x += values[0];
        if(fabs(values[1]) > 1e-3) init_params.lidar_rear_top.transform.y += values[1];
        if(fabs(values[2]) > 1e-3) init_params.lidar_rear_top.transform.z += values[2];
    } else if (lidar_name == init_params.lidar_rear_center.sensor_name) {
        if(fabs(values[0]) > 1e-3) init_params.lidar_rear_center.transform.x += values[0];
        if(fabs(values[1]) > 1e-3) init_params.lidar_rear_center.transform.y += values[1];
        if(fabs(values[2]) > 1e-3) init_params.lidar_rear_center.transform.z += values[2];
    }
}

void rclcomm::setRotateModifyByName(const std::string& lidar_name, const std::vector<double>& values)
{
    if (values.size() != 3) return; // 确保传入的values大小正确

    if (lidar_name == init_params.lidar_front_top.sensor_name) 
    {
        if(fabs(values[0]) > 1e-3) init_params.lidar_front_top.transform.roll += values[0]*M_PI/180.0;
        if(fabs(values[1]) > 1e-3) init_params.lidar_front_top.transform.pitch += values[1]*M_PI/180.0;
        if(fabs(values[2]) > 1e-3) init_params.lidar_front_top.transform.yaw += values[2]*M_PI/180.0;
    } 
    else if (lidar_name == init_params.lidar_front_right.sensor_name) 
    {
        if(fabs(values[0]) > 1e-3) init_params.lidar_front_right.transform.roll += values[0]*M_PI/180.0;
        if(fabs(values[1]) > 1e-3) init_params.lidar_front_right.transform.pitch += values[1]*M_PI/180.0;
        if(fabs(values[2]) > 1e-3) init_params.lidar_front_right.transform.yaw += values[2]*M_PI/180.0;
    } 
    else if (lidar_name == init_params.lidar_front_left.sensor_name) 
    {
        if(fabs(values[0]) > 1e-3) init_params.lidar_front_left.transform.roll += values[0]*M_PI/180.0;
        if(fabs(values[1]) > 1e-3) init_params.lidar_front_left.transform.pitch += values[1]*M_PI/180.0;
        if(fabs(values[2]) > 1e-3) init_params.lidar_front_left.transform.yaw += values[2]*M_PI/180.0;
    } 
    else if (lidar_name == init_params.lidar_rear_top.sensor_name) 
    {
        if(fabs(values[0]) > 1e-3) init_params.lidar_rear_top.transform.roll += values[0]*M_PI/180.0;
        if(fabs(values[1]) > 1e-3) init_params.lidar_rear_top.transform.pitch += values[1]*M_PI/180.0;
        if(fabs(values[2]) > 1e-3) init_params.lidar_rear_top.transform.yaw += values[2]*M_PI/180.0;
    } 
    else if (lidar_name == init_params.lidar_rear_center.sensor_name) 
    {
        if(fabs(values[0]) > 1e-3) init_params.lidar_rear_center.transform.roll += values[0]*M_PI/180.0;
        if(fabs(values[1]) > 1e-3) init_params.lidar_rear_center.transform.pitch += values[1]*M_PI/180.0;
        if(fabs(values[2]) > 1e-3) init_params.lidar_rear_center.transform.yaw += values[2]*M_PI/180.0;
    }
}

bool rclcomm::readParamsFromJson(const std::string& file_path, InitParams& params)
{
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << file_path << std::endl;
        std::cout << "Current path: " << std::filesystem::current_path() << std::endl;
        return false;
    }
    
    json j;
    try {
        ifs >> j;

        // 解析sensor.index
        if (j.contains("sensor") && j["sensor"].contains("index")) {
            auto& index = j["sensor"]["index"];
            params.sensor_index.num = index.value("num", 0);
            params.sensor_index.sensor_frame = index.value("sensor_frame", "");
            params.sensor_index.calibration_file_path = index.value("calibration file path", "");
        }

        // 解析sensor.lidars
        if (j.contains("sensor") && j["sensor"].contains("lidars")) {
            auto& lidars = j["sensor"]["lidars"];
            
            params.lidar_front_top.frame_id = lidars["front_top"]["frame_id"];
            params.lidar_front_top.ros_topic = lidars["front_top"]["ros_topic"];
            params.lidar_front_top.sensor_name = lidars["front_top"]["sensor_name"];

            params.lidar_front_left.frame_id = lidars["front_left"]["frame_id"];
            params.lidar_front_left.ros_topic = lidars["front_left"]["ros_topic"];
            params.lidar_front_left.sensor_name = lidars["front_left"]["sensor_name"];

            params.lidar_front_right.frame_id = lidars["front_right"]["frame_id"];
            params.lidar_front_right.ros_topic = lidars["front_right"]["ros_topic"];
            params.lidar_front_right.sensor_name = lidars["front_right"]["sensor_name"];

            params.lidar_rear_center.frame_id = lidars["rear_center"]["frame_id"];
            params.lidar_rear_center.ros_topic = lidars["rear_center"]["ros_topic"];
            params.lidar_rear_center.sensor_name = lidars["rear_center"]["sensor_name"];

            params.lidar_rear_top.frame_id = lidars["rear_top"]["frame_id"];
            params.lidar_rear_top.ros_topic = lidars["rear_top"]["ros_topic"];
            params.lidar_rear_top.sensor_name = lidars["rear_top"]["sensor_name"];
        }

        // 解析地面区域信息
        if (j.contains("region_ground")) {

            phraseRegionGround(j["region_ground"], params.lidar_front_top);
            phraseRegionGround(j["region_ground"], params.lidar_front_right);
            phraseRegionGround(j["region_ground"], params.lidar_front_left);
            phraseRegionGround(j["region_ground"], params.lidar_rear_top);
            phraseRegionGround(j["region_ground"], params.lidar_rear_center);

            std::cout<< "Region ground for front_top: " << params.lidar_front_top.region_ground.size() << " points." <<params.lidar_front_top.region_ground.at(0).x << std::endl;
            std::cout<< "Region ground for front_right: " << params.lidar_front_right.region_ground.size() << " points." << std::endl;
            std::cout<< "Region ground for front_left: " << params.lidar_front_left.region_ground.size() << " points." << std::endl;
            std::cout<< "Region ground for rear_top: " << params.lidar_rear_top.region_ground.size() << " points." << std::endl;
            std::cout<< "Region ground for rear_center: " << params.lidar_rear_center.region_ground.size() << " points." << std::endl;
        }

        // 解析params部分（如果存在的话）
        if (j.contains("region_wall_x")) {
            phraseRegionWall(j, "region_wall_x", params.lidar_front_top);
            phraseRegionWall(j, "region_wall_x", params.lidar_front_right);
            phraseRegionWall(j, "region_wall_x", params.lidar_front_left);
            phraseRegionWall(j, "region_wall_x", params.lidar_rear_top);
            phraseRegionWall(j, "region_wall_x", params.lidar_rear_center);

            std::cout<< "Region wall for front_top: " << params.lidar_front_top.region_wall_x.size() << " points." <<params.lidar_front_top.region_wall_x.at(0).x << std::endl;
            std::cout<< "Region wall for front_right: " << params.lidar_front_right.region_wall_x.size() << " points." << std::endl;
            std::cout<< "Region wall for front_left: " << params.lidar_front_left.region_wall_x.size() << " points." << std::endl;
            std::cout<< "Region wall for rear_top: " << params.lidar_rear_top.region_wall_x.size() << " points." << std::endl;
            std::cout<< "Region wall for rear_center: " << params.lidar_rear_center.region_wall_x.size() << " points." << std::endl;
        }

        // 解析region_wall_y 墙面Y轴标定
        if (j.contains("region_wall_y")) {
            phraseRegionWall(j, "region_wall_y", params.lidar_front_top);
            phraseRegionWall(j, "region_wall_y", params.lidar_front_right);
            phraseRegionWall(j, "region_wall_y", params.lidar_front_left);
            phraseRegionWall(j, "region_wall_y", params.lidar_rear_top);
            phraseRegionWall(j, "region_wall_y", params.lidar_rear_center);
        }

        if(j.contains("qos"))
        {
            params.qos = QOS_Level::SYSTEM_DEFAULT;
            std::string level = j["qos"]; 

            if(level == "best_effort")
            {
                params.qos = QOS_Level::BEST_EFFORT;
            }
            else if(level == "reliable")
            {
                params.qos = QOS_Level::RELIABLE;
            }
        }


    } catch (const std::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void rclcomm::phraseRegionGround(const json& region_ground, LidarConfig& lidar_config)
{
    if (region_ground.contains(lidar_config.sensor_name)) {
        const auto& plate = region_ground[lidar_config.sensor_name];
        int num = plate.value("num", 0);
        float size = plate.value("size", 0.0f);
        lidar_config.region_ground.resize(num);

        std::string coord_str = plate.value("coordinate", "");
        // 正则表达式提取所有 (x,y,z)
        std::regex re(R"(\(([^,]+),([^,]+),([^)]+)\))");
        std::smatch match;
        auto it = coord_str.cbegin();
        int idx = 0;
        while (std::regex_search(it, coord_str.cend(), match, re) && idx < num) {
            float x = std::stof(match[1].str());
            float y = std::stof(match[2].str());
            float z = std::stof(match[3].str());
            lidar_config.region_ground[idx].x = x;
            lidar_config.region_ground[idx].y = y;
            lidar_config.region_ground[idx].z = z;
            lidar_config.region_ground[idx].intensity = size;   //这里存放的是区域的边长
            it = match.suffix().first;
            ++idx;
        }
    }
}

void rclcomm::phraseRegionWall(const json& json_, const std::string& region_wall_label, LidarConfig& lidar_config)
{
    // 检查json_是否包含region_wall_x
    if (!json_.contains(region_wall_label)) {
        std::cerr << "JSON does not contain region_wall_ for " << region_wall_label << std::endl;
        return;
    }
     
    if("region_wall_x" == region_wall_label) 
    {
        auto region_wall_x = json_[region_wall_label];
        if (region_wall_x.contains(lidar_config.sensor_name)) 
        {
            const auto& plate = region_wall_x[lidar_config.sensor_name];
            int num = plate.value("num", 0);
            float size = plate.value("size", 0.0f);
            lidar_config.region_wall_x.resize(num);

            std::string coord_str = plate.value("coordinate", "");
            // 正则表达式提取所有 (x,y,z)
            std::regex re(R"(\(([^,]+),([^,]+),([^)]+)\))");
            std::smatch match;
            auto it = coord_str.cbegin();
            int idx = 0;
            while (std::regex_search(it, coord_str.cend(), match, re) && idx < num) 
            {
                float x = std::stof(match[1].str());
                float y = std::stof(match[2].str());
                float z = std::stof(match[3].str());
                lidar_config.region_wall_x[idx].x = x;
                lidar_config.region_wall_x[idx].y = y;
                lidar_config.region_wall_x[idx].z = z;
                lidar_config.region_wall_x[idx].intensity = size;   //这里存放的是区域的边长
                it = match.suffix().first;
                ++idx;
            }
        }
    }
    else if("region_wall_y" == region_wall_label)
    {
        auto region_wall_y = json_[region_wall_label];
        if (region_wall_y.contains(lidar_config.sensor_name)) 
        {
            const auto& plate = region_wall_y[lidar_config.sensor_name];
            int num = plate.value("num", 0);
            float size = plate.value("size", 0.0f);
            lidar_config.region_wall_y.resize(num);

            std::string coord_str = plate.value("coordinate", "");
            // 正则表达式提取所有 (x,y,z)
            std::regex re(R"(\(([^,]+),([^,]+),([^)]+)\))");
            std::smatch match;
            auto it = coord_str.cbegin();
            int idx = 0;
            while (std::regex_search(it, coord_str.cend(), match, re) && idx < num) 
            {
                float x = std::stof(match[1].str());
                float y = std::stof(match[2].str());
                float z = std::stof(match[3].str());
                lidar_config.region_wall_y[idx].x = x;
                lidar_config.region_wall_y[idx].y = y;
                lidar_config.region_wall_y[idx].z = z;
                lidar_config.region_wall_y[idx].intensity = size;   //这里存放的是区域的边长
                it = match.suffix().first;
                ++idx;
            }
        }
    }

}

void rclcomm::phraseStereoPlate(const json& intensity_plates, LidarConfig& lidar_config)
{
    if (intensity_plates.contains(lidar_config.sensor_name)) {
        const auto& plate = intensity_plates[lidar_config.sensor_name];
        /*
         *  "size_min_x": 0.2,
            "size_min_y": 0.2,
            "size_max_z": 0.04
         */
        float min_intensity = plate.value("min_intensity", 200.0);
        float max_intensity = plate.value("max_intensity", 255.0);
        float size_min_x = plate.value("size_min_x", 0.2);
        float size_min_y = plate.value("size_min_y", 0.2);
        float size_max_z = plate.value("size_max_z", 0.04);

        lidar_config.plates_stereo.min_intensity = min_intensity;
        lidar_config.plates_stereo.max_intensity = max_intensity;
        lidar_config.plates_stereo.size_min_x = size_min_x;
        lidar_config.plates_stereo.size_min_y = size_min_y;
        lidar_config.plates_stereo.size_max_z = size_max_z;
    }
}

bool rclcomm::readCalibrationFromYaml(const std::string& file_path, InitParams& params)
{
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        
        if (!config["sensor_kit_base_link"]) {
            std::cerr << "No sensor_kit_base_link section found in YAML file" << std::endl;
            return false;
        }
        
        YAML::Node sensor_kit = config["sensor_kit_base_link"];
        
        // 定义雷达frame_id到配置对象的映射
        std::map<std::string, LidarConfig*> lidar_map = {
            {"lidar_ft_base_link", &params.lidar_front_top},
            {"lidar_fr_base_link", &params.lidar_front_right},
            {"lidar_fl_base_link", &params.lidar_front_left},
            {"lidar_rt_base_link", &params.lidar_rear_top},
            {"lidar_rear_base_link", &params.lidar_rear_center}
        };
        
        // 遍历所有雷达配置
        for (const auto& lidar_pair : lidar_map) {
            const std::string& frame_id = lidar_pair.first;
            LidarConfig* lidar_config = lidar_pair.second;
            
            if (sensor_kit[frame_id]) {
                YAML::Node transform_node = sensor_kit[frame_id];
                
                // 读取位姿信息
                double x = transform_node["x"].as<double>();
                double y = transform_node["y"].as<double>();
                double z = transform_node["z"].as<double>();
                double roll = transform_node["roll"].as<double>();
                double pitch = transform_node["pitch"].as<double>();
                double yaw = transform_node["yaw"].as<double>();
                
                // 存储原始Transform信息
                lidar_config->transform = Transform(x, y, z, roll, pitch, yaw);
                
                std::cout << "Loaded calibration for " << frame_id << ": "
                          << "x=" << x << ", y=" << y << ", z=" << z << ", "
                          << "roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << std::endl;
            } else {
                std::cout << "Warning: No calibration data found for " << frame_id << std::endl;
            }
        }
        
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error reading calibration file: " << e.what() << std::endl;
        return false;
    }
}

bool rclcomm::writeYamlFromParams(const std::string& file_path, const InitParams& params)
{
    try {
        YAML::Node config;
        
        // 尝试读取现有文件
        if (std::filesystem::exists(file_path)) {
            try {
                config = YAML::LoadFile(file_path);
                std::cout << "Loading existing YAML file for update..." << std::endl;
            } catch (const YAML::Exception& e) {
                std::cout << "Warning: Existing file has invalid YAML format, creating new file..." << std::endl;
                config = YAML::Node();
            }
        } else {
            std::cout << "Creating new YAML file..." << std::endl;
            config = YAML::Node();
        }
        
        // 确保sensor_kit_base_link节点存在
        if (!config["sensor_kit_base_link"]) {
            config["sensor_kit_base_link"] = YAML::Node();
        }
        
        YAML::Node sensor_kit = config["sensor_kit_base_link"];
        
        // 定义雷达配置到frame_id的映射
        std::map<std::string, const LidarConfig*> lidar_map = {
            {"lidar_ft_base_link", &params.lidar_front_top},
            {"lidar_fr_base_link", &params.lidar_front_right},
            {"lidar_fl_base_link", &params.lidar_front_left},
            {"lidar_rt_base_link", &params.lidar_rear_top},
            {"lidar_rear_base_link", &params.lidar_rear_center}
        };
        
        // 遍历所有雷达配置，写入或更新数据
        for (const auto& lidar_pair : lidar_map) {
            const std::string& frame_id = lidar_pair.first;
            const LidarConfig* lidar_config = lidar_pair.second;
            
            // 创建或更新该雷达的配置节点
            YAML::Node transform_node = sensor_kit[frame_id];
            
            // 写入位姿信息
            transform_node["x"] = lidar_config->transform.x;
            transform_node["y"] = lidar_config->transform.y;
            transform_node["z"] = lidar_config->transform.z;
            transform_node["roll"] = lidar_config->transform.roll;
            transform_node["pitch"] = lidar_config->transform.pitch;
            transform_node["yaw"] = lidar_config->transform.yaw;
            
            // 将更新后的节点写回到sensor_kit节点
            sensor_kit[frame_id] = transform_node;
            
            std::cout << "Writing calibration for " << frame_id << ": "
                      << "x=" << lidar_config->transform.x << ", y=" << lidar_config->transform.y 
                      << ", z=" << lidar_config->transform.z << ", "
                      << "roll=" << lidar_config->transform.roll << ", pitch=" << lidar_config->transform.pitch 
                      << ", yaw=" << lidar_config->transform.yaw << std::endl;
        }
        
        // 将更新后的配置写回文件
        std::ofstream file_out(file_path);
        if (!file_out.is_open()) {
            std::cerr << "Error: Could not open file for writing: " << file_path << std::endl;
            return false;
        }
        
        file_out << config;
        file_out.close();
        
        std::cout << "Successfully wrote calibration data to: " << file_path << std::endl;
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML write error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error writing calibration file: " << e.what() << std::endl;
        return false;
    }
}

// 线程池初始化
void rclcomm::initializeThreadPool()
{
    shutdown_flag_ = false;
    
    // 创建工作线程
    for (size_t i = 0; i < THREAD_POOL_SIZE; ++i) {
        worker_threads_.emplace_back(&rclcomm::workerThread, this);
    }
    
    std::cout << "Thread pool initialized with " << THREAD_POOL_SIZE << " threads." << std::endl;
}

// 关闭线程池
void rclcomm::shutdownThreadPool()
{
    // 设置关闭标志
    shutdown_flag_ = true;
    
    // 唤醒所有工作线程
    queue_condition_.notify_all();
    
    // 等待所有线程结束
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    worker_threads_.clear();
    std::cout << "Thread pool shutdown completed." << std::endl;
}

// 工作线程函数
void rclcomm::workerThread()
{
    while (!shutdown_flag_) {
        PointCloudTask task(nullptr, "");
        
        // 从队列中获取任务
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this] {
                return !task_queue_.empty() || shutdown_flag_;
            });
            
            if (shutdown_flag_) {
                break;
            }
            
            if (!task_queue_.empty()) {
                task = task_queue_.front();
                task_queue_.pop();
            }
        }
        
        // 处理任务
        if (task.msg != nullptr) {
            processPointCloudTask(task);
        }
    }
}

// 处理点云任务
void rclcomm::processPointCloudTask(const PointCloudTask& task)
{
    try {
        if (task.lidar_type == "front_top") {
            processLidarFrontTop(task.msg);
        } else if (task.lidar_type == "front_right") {
            processLidarFrontRight(task.msg);
        } else if (task.lidar_type == "front_left") {
            processLidarFrontLeft(task.msg);
        } else if (task.lidar_type == "rear_top") {
            processLidarRearTop(task.msg);
        } else if (task.lidar_type == "rear_center") {
            processLidarRearCenter(task.msg);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error processing " << task.lidar_type << " pointcloud: " << e.what() << std::endl;
    }
}

void rclcomm::extractYAxisWallPointsByRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_input, const std::vector<pcl::PointXYZI>& boxes_center, std::vector<PlateInfo>& plates, std::vector<BoxInfo>& boxes)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_region(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_plates;
    PlateParams plate_params;
    plate_params.size_min_x = 0.2;
    plate_params.size_min_y = 0.2;
    plate_params.size_max_z = 0.1;

    extractBoxesPointsByRegion(cloud_input, cloud_region, boxes_center);
    clusterFilter(cloud_region, cloud_clustered, 0.3);
    obbBoxFilter(cloud_clustered, cloud_plates, boxes, plate_params);
    check_instance->setPlatesData(plates, boxes, cloud_plates);   //将过滤完成后的点云簇和包围盒信息存储到Check实例中

    for(auto& box : boxes)  {box.color.r = 0.0f; box.color.g = 0.0f; box.color.b = 1.0f;}
}

// 实际的点云处理函数
void rclcomm::processLidarFrontTop(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_plates;
    pcl::fromROSMsg(*msg, *cloud_raw);

    long points_size = cloud_raw->points.size();
    // std::cout << "Processed front_right point cloud with " << points_size << " points in thread." << std::endl;

    if (points_size == 0) {
        std::cout << "Front right point cloud is empty." << std::endl;
        return;
    }

    // transformPointcloudWithTF2(cloud_raw, cloud_transformed, init_params.lidar_front_top);
    transformPointcloudWithParams(cloud_raw, cloud_transformed, init_params.lidar_front_top);

    PlateParams plate_params;
    std::vector<BoxInfo> boxes_for_y;
    if(switch_ground_wall)
    {
        // 使用region_ground进行 区域平面选择
        extractGroundPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_front_top);
        if(init_params.lidar_front_top.region_ground.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_front_top.region_ground.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }

    }
    else
    {
        // plate_params = init_params.lidar_front_top.plates_stereo;
        // 使用强度过滤进行 区域平面选择
        // extractPlatesPointsByIntens(cloud_transformed, cloud_filtered, plate_params);
        if(init_params.lidar_front_top.region_wall_x.size()>0)    //有在配置文件中读取到区域信息
        {
            plate_params.size_min_x = init_params.lidar_front_top.region_wall_x.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }
        extractWallPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_front_top);
        extractYAxisWallPointsByRegion(cloud_transformed, init_params.lidar_front_top.region_wall_y, check_instance->lidar_front_top_plates.plates_for_y, boxes_for_y);

    }

    clusterFilter(cloud_filtered, cloud_clustered);
    // std::cout<<"Clustered point cloud size: " << cloud_clustered.size() << std::endl;

    std::vector<BoxInfo> boxes;
    obbBoxFilter(cloud_clustered, cloud_plates, boxes, plate_params);

    check_instance->setPlatesData(check_instance->lidar_front_top_plates, boxes, cloud_plates);   //将过滤完成后的点云簇和包围盒信息存储到Check实例中

    if(!boxes_for_y.empty()) boxes.insert(boxes.end(), boxes_for_y.begin(), boxes_for_y.end());  //将y方向的包围盒信息添加到总的包围盒信息中
    boxes_show_in_rviz.at(FRONT_TOP).clear();
    
    if(init_params.lidar_front_top.is_stereo_checked || init_params.lidar_front_top.is_ground_checked)
        boxes_show_in_rviz.at(FRONT_TOP) = boxes;

    // 在这里添加具体的点云处理逻辑
}

void rclcomm::processLidarFrontRight(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_plates;
    pcl::fromROSMsg(*msg, *cloud_raw);

    long points_size = cloud_raw->points.size();
    // std::cout << "Processed front_right point cloud with " << points_size << " points in thread." << std::endl;

    if (points_size == 0) {
        std::cout << "Front right point cloud is empty." << std::endl;
        return;
    }

    // transformPointcloudWithTF2(cloud_raw, cloud_transformed, init_params.lidar_front_right);
    transformPointcloudWithParams(cloud_raw, cloud_transformed, init_params.lidar_front_right);

    PlateParams plate_params;
    std::vector<BoxInfo> boxes_for_y;
    if(switch_ground_wall)
    {
        // 使用region_ground进行 区域平面选择
        extractGroundPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_front_right);
        if(init_params.lidar_front_right.region_ground.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_front_right.region_ground.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }

    }
    else
    {
        // plate_params = init_params.lidar_front_right.plates_stereo;
        // // 使用强度过滤进行 区域平面选择
        // extractPlatesPointsByIntens(cloud_transformed, cloud_filtered, plate_params);
        if(init_params.lidar_front_right.region_wall_x.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_front_right.region_wall_x.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }
        extractWallPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_front_right);
        extractYAxisWallPointsByRegion(cloud_transformed, init_params.lidar_front_right.region_wall_y, check_instance->lidar_front_right_plates.plates_for_y, boxes_for_y);
    }

    clusterFilter(cloud_filtered, cloud_clustered);
    // std::cout<<"Clustered point cloud size: " << cloud_clustered.size() << std::endl;

    std::vector<BoxInfo> boxes;
    obbBoxFilter(cloud_clustered, cloud_plates, boxes, plate_params);

    check_instance->setPlatesData(check_instance->lidar_front_right_plates, boxes, cloud_plates);   //将过滤完成后的点云簇和包围盒信息存储到Check实例中

    if(!boxes_for_y.empty()) boxes.insert(boxes.end(), boxes_for_y.begin(), boxes_for_y.end());  //将y方向的包围盒信息添加到总的包围盒信息中
    boxes_show_in_rviz.at(FRONT_RIGHT).clear();

    if(init_params.lidar_front_right.is_stereo_checked || init_params.lidar_front_right.is_ground_checked)
        boxes_show_in_rviz.at(FRONT_RIGHT) = boxes;

    // publishPointCloud(cloud_transformed, points_pub_, init_params.sensor_index.sensor_frame);
}

void rclcomm::processLidarFrontLeft(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_plates;
    pcl::fromROSMsg(*msg, *cloud_raw);

    long points_size = cloud_raw->points.size();
    // std::cout << "Processed front_right point cloud with " << points_size << " points in thread." << std::endl;

    if (points_size == 0) {
        std::cout << "Front right point cloud is empty." << std::endl;
        return;
    }

    // transformPointcloudWithTF2(cloud_raw, cloud_transformed, init_params.lidar_front_left);
    transformPointcloudWithParams(cloud_raw, cloud_transformed, init_params.lidar_front_left);

    PlateParams plate_params;
    std::vector<BoxInfo> boxes_for_y;
    if(switch_ground_wall)
    {
        // 使用region_ground进行 区域平面选择
        extractGroundPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_front_left);
        if(init_params.lidar_front_left.region_ground.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_front_left.region_ground.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }

    }
    else
    {
        // plate_params = init_params.lidar_front_left.plates_stereo;
        // // 使用强度过滤进行 区域平面选择
        // extractPlatesPointsByIntens(cloud_transformed, cloud_filtered, plate_params);
        if(init_params.lidar_front_left.region_wall_x.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_front_left.region_wall_x.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }
        extractWallPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_front_left);
        extractYAxisWallPointsByRegion(cloud_transformed, init_params.lidar_front_left.region_wall_y, check_instance->lidar_front_left_plates.plates_for_y, boxes_for_y);
    }

    clusterFilter(cloud_filtered, cloud_clustered);
    // std::cout<<"Clustered point cloud size: " << cloud_clustered.size() << std::endl;

    std::vector<BoxInfo> boxes;
    obbBoxFilter(cloud_clustered, cloud_plates, boxes, plate_params);

    check_instance->setPlatesData(check_instance->lidar_front_left_plates, boxes, cloud_plates);   //将过滤完成后的点云簇和包围盒信息存储到Check实例中

    if(!boxes_for_y.empty()) boxes.insert(boxes.end(), boxes_for_y.begin(), boxes_for_y.end());  //将y方向的包围盒信息添加到总的包围盒信息中
    boxes_show_in_rviz.at(FRONT_LEFT).clear();

    if(init_params.lidar_front_left.is_stereo_checked || init_params.lidar_front_left.is_ground_checked)
        boxes_show_in_rviz.at(FRONT_LEFT) = boxes;

}

void rclcomm::processLidarRearTop(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_plates;
    pcl::fromROSMsg(*msg, *cloud_raw);

    long points_size = cloud_raw->points.size();
    // std::cout << "Processed front_right point cloud with " << points_size << " points in thread." << std::endl;

    if (points_size == 0) {
        std::cout << "Front right point cloud is empty." << std::endl;
        return;
    }

    // transformPointcloudWithTF2(cloud_raw, cloud_transformed, init_params.lidar_rear_top);
    transformPointcloudWithParams(cloud_raw, cloud_transformed, init_params.lidar_rear_top);

    PlateParams plate_params;
    std::vector<BoxInfo> boxes_for_y;
    if(switch_ground_wall)
    {
        // 使用region_ground进行 区域平面选择
        extractGroundPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_rear_top);
        if(init_params.lidar_rear_top.region_ground.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_rear_top.region_ground.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }

    }
    else
    {
        // plate_params = init_params.lidar_rear_top.plates_stereo;
        // // 使用强度过滤进行 区域平面选择
        // extractPlatesPointsByIntens(cloud_transformed, cloud_filtered, plate_params);
        if(init_params.lidar_rear_top.region_wall_x.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_rear_top.region_wall_x.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }
        extractWallPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_rear_top);
        
        extractYAxisWallPointsByRegion(cloud_transformed, init_params.lidar_rear_top.region_wall_y, check_instance->lidar_rear_top_plates.plates_for_y, boxes_for_y);
    }

    clusterFilter(cloud_filtered, cloud_clustered);
    // std::cout<<"Clustered point cloud size: " << cloud_clustered.size() << std::endl;

    std::vector<BoxInfo> boxes;
    obbBoxFilter(cloud_clustered, cloud_plates, boxes, plate_params);

    check_instance->setPlatesData(check_instance->lidar_rear_top_plates, boxes, cloud_plates);   //将过滤完成后的点云簇和包围盒信息存储到Check实例中

    if(!boxes_for_y.empty()) boxes.insert(boxes.end(), boxes_for_y.begin(), boxes_for_y.end());  //将y方向的包围盒信息添加到总的包围盒信息中
    boxes_show_in_rviz.at(REAR_TOP).clear();

    if(init_params.lidar_rear_top.is_stereo_checked || init_params.lidar_rear_top.is_ground_checked)
        boxes_show_in_rviz.at(REAR_TOP) = boxes;

}

void rclcomm::processLidarRearCenter(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_clustered;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_plates;
    pcl::fromROSMsg(*msg, *cloud_raw);

    long points_size = cloud_raw->points.size();
    // std::cout << "Processed front_right point cloud with " << points_size << " points in thread." << std::endl;

    if (points_size == 0) {
        std::cout << "Front right point cloud is empty." << std::endl;
        return;
    }

    // transformPointcloudWithTF2(cloud_raw, cloud_transformed, init_params.lidar_rear_center);
    transformPointcloudWithParams(cloud_raw, cloud_transformed, init_params.lidar_rear_center);

    PlateParams plate_params;
    std::vector<BoxInfo> boxes_for_y;
    if(switch_ground_wall)
    {
        // 使用region_ground进行 区域平面选择
        extractGroundPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_rear_center);
        if(init_params.lidar_rear_center.region_ground.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_rear_center.region_ground.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }

    }
    else
    {
        // plate_params = init_params.lidar_rear_center.plates_stereo;
        // // 使用强度过滤进行 区域平面选择
        // extractPlatesPointsByIntens(cloud_transformed, cloud_filtered, plate_params);
        if(init_params.lidar_rear_center.region_wall_x.size()>0)
        {
            plate_params.size_min_x = init_params.lidar_rear_center.region_wall_x.at(0).intensity / 2.0;
            plate_params.size_min_y = plate_params.size_min_x ;
            plate_params.size_max_z = 0.1;
        }
        else
        {
            plate_params.size_min_x=0.2;plate_params.size_min_y=0.2;plate_params.size_max_z=0.1;
        }
        extractWallPointsByRegion(cloud_transformed, cloud_filtered, init_params.lidar_rear_center);
        extractYAxisWallPointsByRegion(cloud_transformed, init_params.lidar_rear_center.region_wall_y, check_instance->lidar_rear_center_plates.plates_for_y, boxes_for_y);
    }

    clusterFilter(cloud_filtered, cloud_clustered);
    // std::cout<<"Clustered point cloud size: " << cloud_clustered.size() << std::endl;

    std::vector<BoxInfo> boxes;
    obbBoxFilter(cloud_clustered, cloud_plates, boxes, plate_params);

    check_instance->setPlatesData(check_instance->lidar_rear_center_plates, boxes, cloud_plates);   //将过滤完成后的点云簇和包围盒信息存储到Check实例中

    if(!boxes_for_y.empty()) boxes.insert(boxes.end(), boxes_for_y.begin(), boxes_for_y.end());  //将y方向的包围盒信息添加到总的包围盒信息中
    boxes_show_in_rviz.at(REAR_CENTER).clear();

    if(init_params.lidar_rear_center.is_stereo_checked || init_params.lidar_rear_center.is_ground_checked)
        boxes_show_in_rviz.at(REAR_CENTER) = boxes;
}

void rclcomm::publishStaticTfTransforms()
{
    if (!static_tf_broadcaster_ || !node) {
        std::cerr << "Static TF broadcaster or node is not initialized!" << std::endl;
        return;
    }

    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    
    // 获取当前时间戳
    rclcpp::Time now = node->now();
    
    // 定义所有雷达配置的映射
    std::vector<std::pair<std::string, const LidarConfig*>> lidar_configs = {
        {"lidar_ft_base_link", &init_params.lidar_front_top},
        {"lidar_fr_base_link", &init_params.lidar_front_right},
        {"lidar_fl_base_link", &init_params.lidar_front_left},
        {"lidar_rt_base_link", &init_params.lidar_rear_top},
        {"lidar_rear_base_link", &init_params.lidar_rear_center}
    };
    
    // 为每个雷达创建静态变换
    for (const auto& lidar_pair : lidar_configs) {
        const std::string& frame_id = lidar_pair.first;
        const LidarConfig* lidar_config = lidar_pair.second;
        
        geometry_msgs::msg::TransformStamped transform_msg;
        
        // 设置变换消息的头部信息
        transform_msg.header.stamp = now;
        transform_msg.header.frame_id = init_params.sensor_index.sensor_frame; // 父坐标系
        transform_msg.child_frame_id = frame_id; // 子坐标系
        
        // 设置位置信息
        transform_msg.transform.translation.x = lidar_config->transform.x;
        transform_msg.transform.translation.y = lidar_config->transform.y;
        transform_msg.transform.translation.z = lidar_config->transform.z;
        
        // 将欧拉角转换为四元数
        tf2::Quaternion quaternion;
        quaternion.setRPY(
            lidar_config->transform.roll,
            lidar_config->transform.pitch,
            lidar_config->transform.yaw
        );
        
        // 设置旋转信息
        transform_msg.transform.rotation.x = quaternion.x();
        transform_msg.transform.rotation.y = quaternion.y();
        transform_msg.transform.rotation.z = quaternion.z();
        transform_msg.transform.rotation.w = quaternion.w();
        
        static_transforms.push_back(transform_msg);
        
        // std::cout << "Created static transform for " << frame_id 
        //           << " -> " << init_params.sensor_index.sensor_frame
        //           << " at position (" << lidar_config->transform.x << ", "
        //           << lidar_config->transform.y << ", " << lidar_config->transform.z << ")"
        //           << " with rotation (" << lidar_config->transform.roll << ", "
        //           << lidar_config->transform.pitch << ", " << lidar_config->transform.yaw << ")" << std::endl;
    }
    
    // 发布所有静态变换
    if (!static_transforms.empty()) {
        static_tf_broadcaster_->sendTransform(static_transforms);
        // std::cout << "Published " << static_transforms.size() << " static TF transforms." << std::endl;
    } else {
        std::cout << "No static transforms to publish." << std::endl;
    }
}

