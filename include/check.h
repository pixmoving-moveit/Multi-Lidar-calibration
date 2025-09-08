#ifndef CHECK_H
#define CHECK_H

#include <map>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <functional>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <std_msgs/msg/header.hpp>

// Forward declaration
struct InitParams;

enum LidarIndex {
    FRONT_TOP = 0,
    FRONT_RIGHT,
    FRONT_LEFT,
    REAR_TOP,
    REAR_CENTER
};

struct BoxInfo {
    Eigen::Vector3f size;         // 长宽高
    Eigen::Vector3f position;     // 中心位置
    Eigen::Quaternionf orientation; // 姿态
    std_msgs::msg::ColorRGBA color; // 颜色，可选
    BoxInfo() {color.r = 0.0f; color.g = 1.0f; color.b = 0.0f; color.a = 0.5f;} // 默认颜色为半透明绿色
};

struct PlateInfo
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Vector3f position; // 中心位置
    Eigen::Quaternionf orientation; // 姿态
};

struct LidarPlates
{
    std::string lidar_id; // 雷达ID
    std::vector<PlateInfo> plates; // 存储检测到的板信息
    std::vector<PlateInfo> plates_for_y; // 用于Y轴校验的平面信息
    LidarPlates() : lidar_id("unknown") {}
};

struct PlaneAnalysisResult
{
    float z = 0.0f; // 平面方程的z值
    Eigen::Vector3d orientation;    // 平面法向量的方向
    std::string describe; // 描述信息
    PlaneAnalysisResult() : z(0.0f), orientation(Eigen::Vector3d::Zero()), describe("") {}
};

struct PlaneInfo
{
    Eigen::Vector3f centroid;     // 点云质心
    Eigen::Vector4f coefficients; // 平面方程系数 ax + by + cz + d = 0
    int inlier_count;            // 内点数量
    float confidence;            // 拟合置信度
    bool is_valid;               // 是否为有效平面
    
    PlaneInfo() :
                centroid(Eigen::Vector3f::Zero()),
                coefficients(Eigen::Vector4f::Zero()),
                inlier_count(0),
                confidence(0.0f),
                is_valid(false) {}
};


struct PlateParams
{
    float min_intensity;
    float max_intensity;
    float size_min_x;
    float size_min_y;
    float size_max_z;
    PlateParams(): min_intensity(200),max_intensity(255),size_min_x(0.2),size_min_y(0.2),size_max_z(0.04){}
    PlateParams(float min_intensity_, float max_intensity_)
        : min_intensity(min_intensity_), max_intensity(max_intensity_) {}
};

// 位姿变换结构体
struct Transform
{
    double x, y, z;        // 平移
    double roll, pitch, yaw; // 旋转（欧拉角）
    
    Transform() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    Transform(double x_, double y_, double z_, double roll_, double pitch_, double yaw_)
        : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_) {}
};

struct LidarConfig
{
    std::string sensor_name;
    std::string ros_topic;
    std::string frame_id;
    std_msgs::msg::Header header; // 用于存储点云的时间戳和坐标系信息
    std::vector<pcl::PointXYZI> region_ground;   //！！！用于存储region信息，intensity存储size
    std::vector<pcl::PointXYZI> region_wall_x;   //！！！用于存储region信息，intensity存储size
    std::vector<pcl::PointXYZI> region_wall_y;   //！！！用于存储region信息，intensity存储size
    PlateParams plates_stereo;
    Transform transform; // 相对于sensor_kit_base_link的位姿变换（欧拉角形式）

    bool is_active;
    bool is_ground_checked; // 是否被选中
    bool is_stereo_checked; // 是否被选中
    LidarConfig() : is_active(false), is_ground_checked(false), is_stereo_checked(false) {}

    Eigen::Matrix4d getTransMatrix() const  // 通过Transform获取变换矩阵
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        // 旋转部分
        Eigen::AngleAxisd rollAngle(transform.roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(transform.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(transform.yaw, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).matrix();
        mat.block<3,3>(0,0) = rotation;
        // 平移部分
        mat(0,3) = transform.x;
        mat(1,3) = transform.y;
        mat(2,3) = transform.z;
        return mat;
    }

};


class Check {
public:
    Check();
    ~Check();

    std::string groundAutoCali_RP(InitParams& init_params);
    std::string lidargroundAutoCali_RP(LidarConfig& lidar_config);
    std::string groundAutoCali_Z(InitParams& init_params);
    std::string stereoAutoCali_Yaw(InitParams& init_params);
    std::string stereoAutoCali_X(InitParams& init_params);
    std::string stereoAutoCali_Y(InitParams& init_params);

    static Check* getInstance(); 
    void setPlatesData(LidarPlates& lidar_plate, const std::vector<BoxInfo>& boxes, const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_plates);
    void setPlatesData(std::vector<PlateInfo>& plates, const std::vector<BoxInfo>& boxes, const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_plates);

    int getPlatesCount(const std::string& lidar_id) const;
    int getPlates_for_y_Count(const std::string& lidar_id) const;
    
    // 新增函数：根据lidar名称分析对应的LidarPlates数据
    PlaneAnalysisResult analyzeLidarByName(const std::string& lidar_name);
    
    // RANSAC平面拟合函数
    PlaneInfo fitPlaneRANSAC(const pcl::PointCloud<pcl::PointXYZI>& cloud, double distance_threshold = 0.01, int max_iterations = 1000, double probability = 0.99);
    
    // SVD最小二乘平面拟合函数
    PlaneInfo fitPlaneSVD(const pcl::PointCloud<pcl::PointXYZI>& cloud, double distance_threshold = 0.01);

    // 计算重叠区域Z值差异
    std::pair<float, float> calcOverlapDis(const std::vector<PlateInfo>& plates_1, const std::vector<PlateInfo>& plates_2, const std::string& axis);
    Eigen::Vector3d calcOverlapPlates(const LidarPlates& plates_1, const LidarPlates& plates_2);
    Eigen::Vector3d calcOverallPlates(const LidarPlates& plates_1, const LidarPlates& plates_2);
    Eigen::Vector3d calcOverallPlates(const LidarPlates& plates_1, const LidarPlates& plates_2, const LidarPlates& plates_3);

    std::string groundAnalysis();
    std::string stereoAnalysis();
    // lidar名称到分析函数的映射
    
    // 定义子函数类型
    using LidarAnalysisFunc = std::function<PlaneAnalysisResult()>;
    std::map<std::string, LidarAnalysisFunc> lidar_analysis_map;
    Eigen::Vector3d calcRotationEulerZYX(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
    Eigen::Vector3d calcRotationEulerZYX(const PlaneInfo& v1, const PlaneInfo& v2);

    bool ground_cali_done; // fr_fl地面校准是否完成
    bool yaw_cali_done; // fr_fl立体校准是否完成

private:
    
    
    // 初始化lidar名称到分析函数的映射
    void initializeLidarAnalysisMap();
    
    // 各个雷达的具体分析函数
    PlaneAnalysisResult analyzeFrontTop(const bool& details);
    PlaneAnalysisResult analyzeFrontRight(const bool& details);
    PlaneAnalysisResult analyzeFrontLeft(const bool& details);
    PlaneAnalysisResult analyzeRearTop(const bool& details);
    PlaneAnalysisResult analyzeRearCenter(const bool& details);
    PlaneAnalysisResult analyzePlates(const LidarPlates& lidar_plate, const bool& need_details);

    float calc_z_mean(const std::vector<PlateInfo>& plates) const;


public:
    LidarPlates lidar_front_top_plates;
    LidarPlates lidar_front_right_plates;
    LidarPlates lidar_front_left_plates;
    LidarPlates lidar_rear_top_plates;
    LidarPlates lidar_rear_center_plates;

private:
        
    
};

#endif // CHECK_H
