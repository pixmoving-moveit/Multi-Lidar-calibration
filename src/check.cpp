#include "check.h"
#include "rclcomm.h"

Check::Check()
{
    lidar_front_top_plates.lidar_id = "front_top";
    lidar_front_right_plates.lidar_id = "front_right";
    lidar_front_left_plates.lidar_id = "front_left";
    lidar_rear_top_plates.lidar_id = "rear_top";
    lidar_rear_center_plates.lidar_id = "rear_center";

    ground_cali_done = false;
    yaw_cali_done = false;
    
    // 初始化雷达分析函数映射
    initializeLidarAnalysisMap();
}
Check::~Check()
{

}

//地面标定，只计算roll、pitch两个值
std::string Check::groundAutoCali_RP(InitParams& init_params)
{
    std::ostringstream oss;

    // 1. 必须保证front_right和front_left的平面矢量都存在
    if(lidar_front_right_plates.plates.size()>=1 && lidar_front_left_plates.plates.size()>=1)
    {
        oss<< "Front right and front left plates exist, calibration can proceed.";
    }
    else
    {
        oss << "Front right or front left plates not enough for calibration.";
        return oss.str();
    }

    // 2. 分别对每个雷达进行地面标定
    std::string rel = "";
    oss << "Ground calibration results:\n";

    // 对每个雷达进行地面 Roll Pitch标定
    if(init_params.lidar_front_right.is_ground_checked && init_params.lidar_front_left.is_ground_checked)
    {
        if(!ground_cali_done)
        {
            Eigen::Vector3d euler_right_ground, euler_left_ground;
            rel = lidargroundAutoCali_RP(init_params.lidar_front_right);
            oss << rel << "\n";
            rel = lidargroundAutoCali_RP(init_params.lidar_front_left);
            oss << rel << "\n"; 
        }
        else
        {
            oss << "Ground calibration for front right and front left already done.\n";
        }
    
    }

    if(init_params.lidar_front_top.is_ground_checked && lidar_front_top_plates.plates.size() > 0) 
    {
        rel = lidargroundAutoCali_RP(init_params.lidar_front_top);
        oss << rel << "\n";
    }
    if(init_params.lidar_rear_top.is_ground_checked && lidar_rear_top_plates.plates.size() > 0) 
    {
        rel = lidargroundAutoCali_RP(init_params.lidar_rear_top);
        oss << rel << "\n";
    }
    if(init_params.lidar_rear_center.is_ground_checked && lidar_rear_center_plates.plates.size() > 0) 
    {
        rel = lidargroundAutoCali_RP(init_params.lidar_rear_center);
        oss << rel << "\n";
    }
    return oss.str();
}

//地面标定，只计算Z值， 这个必须在groundAutoCali_RP执行之后，且更新到新的平面姿态后执行
std::string Check::groundAutoCali_Z(InitParams& init_params)
{
    std::ostringstream oss;

    float z_mean = (calc_z_mean(lidar_front_right_plates.plates) + calc_z_mean(lidar_front_left_plates.plates)) / 2.0f;

    oss << "Ground Z mean: " << z_mean << "\n";
    

    if(init_params.lidar_front_top.is_ground_checked && lidar_front_top_plates.plates.size() > 0) 
    {
        float z_diff_front_top = calc_z_mean(lidar_front_top_plates.plates) - z_mean; // 计算front_top与地平面的的z差
        init_params.lidar_front_top.transform.z -= z_diff_front_top; // 更新front_top的z值
        oss << "Front Top Z difference: " << z_diff_front_top << "\n";
    }
    if(init_params.lidar_front_right.is_ground_checked && lidar_front_right_plates.plates.size() > 0)
    {
        float z_diff_front_right = calc_z_mean(lidar_front_right_plates.plates) - z_mean; // 计算front_right与地平面的的z差
        init_params.lidar_front_right.transform.z -= z_diff_front_right; // 更新front_right的z值
        oss << "Front Right Z difference: " << z_diff_front_right << "\n";
    }
    if(init_params.lidar_front_left.is_ground_checked && lidar_front_left_plates.plates.size() > 0) 
    {
        float z_diff_front_left = calc_z_mean(lidar_front_left_plates.plates) - z_mean; // 计算front_left与地平面的的z差
        init_params.lidar_front_left.transform.z -= z_diff_front_left; // 更新front_left的z值
        oss << "Front Left Z difference: " << z_diff_front_left << "\n";
    }
    if(init_params.lidar_rear_top.is_ground_checked && lidar_rear_top_plates.plates.size() > 0) 
    {
        float z_diff_rear_top = calc_z_mean(lidar_rear_top_plates.plates) - z_mean; // 计算rear_top与地平面的的z差
        init_params.lidar_rear_top.transform.z -= z_diff_rear_top; // 更新rear_top的z值
        oss << "Rear Top Z difference: " << z_diff_rear_top << "\n";
    }
    if(init_params.lidar_rear_center.is_ground_checked && lidar_rear_center_plates.plates.size() > 0) 
    {
        float z_diff_rear_center = calc_z_mean(lidar_rear_center_plates.plates) - z_mean; // 计算rear_center与地平面的的z差
        init_params.lidar_rear_center.transform.z -= z_diff_rear_center; // 更新rear_center的z值
        oss << "Rear Center Z difference: " << z_diff_rear_center << "\n";
    }
    return oss.str();
}

//立体标定，Yaw值 这个必须在地面标定完成后执行
std::string Check::stereoAutoCali_Yaw(InitParams& init_params)
{
    std::ostringstream oss;
    // 1. 必须保证front_right、front_left都被勾选
    if(init_params.lidar_front_right.is_stereo_checked && init_params.lidar_front_left.is_stereo_checked)
    {
        oss<< "Front right and front left plates exist, calibration can proceed.";
    }
    else
    {
        oss << "Front right and front left is not checked.";
        return oss.str();
    }

    // 2. 分别对每个雷达进行立体标定
    if(init_params.lidar_front_left.is_stereo_checked && lidar_front_left_plates.plates.size() > 0)
    {
        //计算front_left变换到front_right的欧拉角
        if(!yaw_cali_done)
        {
            Eigen::Vector3d angles_fl_fr = calcOverallPlates(lidar_front_left_plates, lidar_front_right_plates);
            init_params.lidar_front_left.transform.yaw += angles_fl_fr[0]; // 更新front_left的yaw值
            oss << "Front Left to Front Right Yaw: " << angles_fl_fr[0] << "\n";
        }
        else
        {
            oss << "Stereo calibration for front right and front left already done.\n";
        }

    }
    if(init_params.lidar_front_top.is_stereo_checked && lidar_front_top_plates.plates.size() > 0) 
    {
        //计算front_top变换到front_right的欧拉角
        Eigen::Vector3d angles_ft_fr = calcOverallPlates(lidar_front_top_plates, lidar_front_right_plates);
        init_params.lidar_front_top.transform.yaw += angles_ft_fr[0]; // 更新front_top的yaw值
        oss << "Front Top to Front Right Yaw: " << angles_ft_fr[0] << "\n";
    }

    if(init_params.lidar_rear_top.is_stereo_checked && lidar_rear_top_plates.plates.size() > 0) 
    {
        if(yaw_cali_done)
        {
            oss << "Stereo calibration for front right and front left already done.\n";
            oss << "Yaw calibration for rear top will be performed.\n";
            //计算rear_top变换到front_right的欧拉角
            if(lidar_front_right_plates.plates.size() > 0)
            {
                Eigen::Vector3d angles_rt_fr = calcOverallPlates(lidar_rear_top_plates, lidar_front_right_plates);
                init_params.lidar_rear_top.transform.yaw += angles_rt_fr[0]; // 更新rear_top的yaw值
                oss << "Rear Top to Front Right Yaw: " << angles_rt_fr[0] << "\n";
            }
            else
            {
                oss << "front_right or front_left haven't enough plates for calibration.\n";
            }
            return oss.str();
        }
        else
        {
            oss << "Stereo calibration for front right and front left not done yet.\n";
            oss << "Yaw calibration for rear top will be skipped.\n";
            return oss.str();
        }
        
    }
    if(init_params.lidar_rear_center.is_stereo_checked && lidar_rear_center_plates.plates.size() > 0) 
    {
        if(yaw_cali_done)
        {
            oss << "Stereo calibration for front right and front left already done.\n";
            oss << "Yaw calibration for rear center will be performed.\n";
            //计算rear_center变换到front_right的欧拉角
            if(lidar_front_right_plates.plates.size() > 0)
            {
                Eigen::Vector3d angles_rc_fr = calcOverallPlates(lidar_rear_center_plates, lidar_front_right_plates);
                init_params.lidar_rear_center.transform.yaw += angles_rc_fr[0]; // 更新rear_center的yaw值
                oss << "Rear Center to Front Right Yaw: " << angles_rc_fr[0] << "\n";
            }
            else
            {
                oss << "front_right or front_left haven't enough plates for calibration.\n";
            }
            return oss.str();
        }
        else
        {
            oss << "Stereo calibration for front right and front left not done yet.\n";
            oss << "Yaw calibration for rear center will be skipped.\n";
            return oss.str();
        }
    }

    return oss.str();
}


std::string Check::stereoAutoCali_X(InitParams& init_params)
{
    std::ostringstream oss;

    // 1. 必须保证front_right的平面矢量存在，且大于2个
    if(lidar_front_right_plates.plates.size()>=2 && init_params.lidar_front_right.is_stereo_checked)
    {
        oss<< "Front right and front left plates exist, X calibration can proceed.";
        
    }
    else
    {
        oss << "Front right or front left plates not enough for X calibration.";
        return oss.str();
    }

    

    // 2. 分别对每个雷达X进行立体标定
    if(init_params.lidar_front_left.is_stereo_checked && lidar_front_left_plates.plates.size() > 0)
    {
        //计算front_left变换到front_right的X轴差值
        std::pair<float, float> dis_fl_fr = calcOverlapDis(lidar_front_left_plates.plates, lidar_front_right_plates.plates, "x");
        init_params.lidar_front_left.transform.x -= dis_fl_fr.first; // 更新front_left的x值
        oss << "Front Left to Front Right X: " << dis_fl_fr.first << "\n";
    }
    if(init_params.lidar_front_top.is_stereo_checked && lidar_front_top_plates.plates.size() > 0) 
    {
        //计算front_top变换到front_right的X轴差值
        std::pair<float, float> dis_ft_fr = calcOverlapDis(lidar_front_top_plates.plates, lidar_front_right_plates.plates, "x");
        init_params.lidar_front_top.transform.x -= dis_ft_fr.first; // 更新front_top的x值
        oss << "Front Top to Front Right X: " << dis_ft_fr.first << "\n";
    }
    if(init_params.lidar_rear_top.is_stereo_checked && lidar_rear_top_plates.plates.size() > 0) 
    {
        //计算rear_top变换到front_right的X轴差值
        std::pair<float, float> dis_rt_fr = calcOverlapDis(lidar_rear_top_plates.plates, lidar_front_right_plates.plates, "x");
        init_params.lidar_rear_top.transform.x -= dis_rt_fr.first; // 更新rear_top的x值
        oss << "Rear Top to Front Right X: " << dis_rt_fr.first << "\n";
    }
    if(init_params.lidar_rear_center.is_stereo_checked && lidar_rear_center_plates.plates.size() > 0) 
    {
        //计算rear_center变换到front_right的X轴差值
        std::pair<float, float> dis_rc_fr = calcOverlapDis(lidar_rear_center_plates.plates, lidar_front_right_plates.plates, "x");
        init_params.lidar_rear_center.transform.x -= dis_rc_fr.first; // 更新rear_center的x值
        oss << "Rear Center to Front Right X: " << dis_rc_fr.first << "\n";
    }

    return oss.str();
}

std::string Check::stereoAutoCali_Y(InitParams& init_params)
{
    std::ostringstream oss;

    // 1. 必须保证front_right的平面矢量存在，且大于2个
    if(lidar_front_right_plates.plates_for_y.size()>=1 && init_params.lidar_front_right.is_stereo_checked)
    {
        oss<< "Front right and front left plates exist, Y calibration can proceed.";
        
    }
    else
    {
        oss << "Front right or front left plates not enough for Y calibration.";
        return oss.str();
    }
    

    // 2. 分别对每个雷达Y进行立体标定
    if(init_params.lidar_front_left.is_stereo_checked && lidar_front_left_plates.plates_for_y.size() > 0)
    {
        //计算front_left变换到front_right的Y轴差值
        std::pair<float, float> dis_fl_fr = calcOverlapDis(lidar_front_left_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
        init_params.lidar_front_left.transform.y -= dis_fl_fr.first; // 更新front_left的y值
        oss << "Front Left to Front Right Y: " << dis_fl_fr.first << "\n";
    }
    if(init_params.lidar_front_top.is_stereo_checked && lidar_front_top_plates.plates_for_y.size() > 0) 
    {
        //计算front_top变换到front_right的Y轴差值
        std::pair<float, float> dis_ft_fr = calcOverlapDis(lidar_front_top_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
        init_params.lidar_front_top.transform.y -= dis_ft_fr.first; // 更新front_top的y值
        oss << "Front Top to Front Right Y: " << dis_ft_fr.first << "\n";
    }
    if(init_params.lidar_rear_top.is_stereo_checked && lidar_rear_top_plates.plates_for_y.size() > 0) 
    {
        //计算rear_top变换到front_right的Y轴差值
        std::pair<float, float> dis_rt_fr = calcOverlapDis(lidar_rear_top_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
        init_params.lidar_rear_top.transform.y -= dis_rt_fr.first; // 更新rear_top的y值
        oss << "Rear Top to Front Right Y: " << dis_rt_fr.first << "\n";
    }
    if(init_params.lidar_rear_center.is_stereo_checked && lidar_rear_center_plates.plates_for_y.size() > 0) 
    {
        //计算rear_center变换到front_right的Y轴差值
        std::pair<float, float> dis_rc_fr = calcOverlapDis(lidar_rear_center_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
        init_params.lidar_rear_center.transform.y -= dis_rc_fr.first; // 更新rear_center的y值
        oss << "Rear Center to Front Right Y: " << dis_rc_fr.first << "\n";
    }

    return oss.str();
}


// 对单个雷达进行地面标定 封装函数，便于调用
std::string Check::lidargroundAutoCali_RP(LidarConfig& lidar_config)
{
    if(!lidar_config.is_active) {
        return "Lidar " + lidar_config.sensor_name + " is not active.";
    }
    // 直接引用，确保修改会影响lidar_config
    PlaneAnalysisResult plane_lidar = analyzeLidarByName(lidar_config.sensor_name);
    Eigen::Vector3d orientation_ground = Eigen::Vector3d(0.0,0.0,1.0);  //设置地平面矢量方向为(0,0,1)
    Eigen::Vector3d angles_diff = calcRotationEulerZYX(plane_lidar.orientation, orientation_ground); //计算雷达平面变换到地平面的欧拉角

    // 检查欧拉角差异是否足够小,过小的话就不再进行坐标变换了
    if(angles_diff.isMuchSmallerThan(1.0, 1e-5))
    {
        return "Lidar " + lidar_config.sensor_name + " is already aligned with ground.";
    }

    Eigen::Matrix3d rotation_matrix;    //将欧拉角转化为旋转矩阵
    rotation_matrix = Eigen::AngleAxisd(angles_diff[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(angles_diff[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(angles_diff[2], Eigen::Vector3d::UnitX());
    
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();    //构建4x4变换矩阵
    transform.block<3,3>(0,0) = rotation_matrix;

    Eigen::Matrix4d T = lidar_config.getTransMatrix();  //取原始雷达到sensor_kit_base_link的变换矩阵

    Eigen::Matrix4d T2 = transform * T; //计算新的变换矩阵

    Eigen::Matrix3d rotation = T2.block<3,3>(0,0);  //提取旋转部分
    Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0); // 计算新的雷达到sensor_kit_base_link的欧拉角

    lidar_config.transform.roll = euler[2];
    lidar_config.transform.pitch = euler[1];   // 更新雷达到sensor_kit_base_link的欧拉角
    lidar_config.transform.yaw = euler[0];   // 更新雷达到sensor_kit_base_link的欧拉角

    return "Lidar " + lidar_config.sensor_name + " calibration completed: "
           + "roll=" + std::to_string(euler[2]) + ", "
           + "pitch=" + std::to_string(euler[1]) + ", "
           + "yaw=" + std::to_string(euler[0]);
}


std::string Check::groundAnalysis()
{
    std::ostringstream oss;

    // 1. fr-fl, fr-rc, fl-rc计算重叠区域z值

    float z_mean = (calc_z_mean(lidar_front_right_plates.plates) + calc_z_mean(lidar_front_left_plates.plates)) / 2.0f;

    float z_diff_front_top = calc_z_mean(lidar_front_top_plates.plates) - z_mean; // 计算front_top与地平面的的z差
    float z_diff_front_right = calc_z_mean(lidar_front_right_plates.plates) - z_mean; // 计算front_right与地平面的的z差
    float z_diff_front_left = calc_z_mean(lidar_front_left_plates.plates) - z_mean; // 计算front_left与地平面的的z差
    float z_diff_rear_top = calc_z_mean(lidar_rear_top_plates.plates) - z_mean; // 计算rear_top与地平面的的z差
    float z_diff_rear_center = calc_z_mean(lidar_rear_center_plates.plates) - z_mean; // 计算rear_center与地平面的的z差


    oss << "groundAnalysis[1]: Caculate difference between Lidar and Ground ----->  \n" ;
    oss << "ground z_mean = " << z_mean << "\n";
    oss << "ground z_diff_front_top = " << z_diff_front_top << "\n";
    oss << "ground z_diff_front_right = " << z_diff_front_right << "\n";
    oss << "ground z_diff_front_left = " << z_diff_front_left << "\n";
    oss << "ground z_diff_rear_top = " << z_diff_rear_top << "\n";
    oss << "ground z_diff_rear_center = " << z_diff_rear_center << "\n";

    // 2. 计算ft, fr, fl, rt, rc五个雷达的平面矢量

    PlaneAnalysisResult ft = analyzeFrontTop(false);
    PlaneAnalysisResult fr = analyzeFrontRight(false);
    PlaneAnalysisResult fl = analyzeFrontLeft(false);
    PlaneAnalysisResult rt = analyzeRearTop(false);
    PlaneAnalysisResult rc = analyzeRearCenter(false);

    oss << "groundAnalysis[2]: Caculate each lidar's ground normal vector----->  \n" ;
    oss << ft.describe;
    oss << fr.describe;
    oss << fl.describe;
    oss << rt.describe;
    oss << rc.describe;

    // Eigen::Vector3d orientation_ground = (fl.orientation + fr.orientation)*0.5;  //fr fl两个雷达所代表的地平面
    Eigen::Vector3d orientation_ground = Eigen::Vector3d(0.0,0.0,1.0);  //地平面
    Eigen::Vector3d angles_fr_fl = calcRotationEulerZYX(fr.orientation, fl.orientation);
    Eigen::Vector3d angles_diff_fr = calcRotationEulerZYX(fr.orientation, orientation_ground);
    Eigen::Vector3d angles_diff_fl = calcRotationEulerZYX(fl.orientation, orientation_ground);
    Eigen::Vector3d angles_diff_ft = calcRotationEulerZYX(ft.orientation, orientation_ground);
    Eigen::Vector3d angles_diff_rt = calcRotationEulerZYX(rt.orientation, orientation_ground);
    Eigen::Vector3d angles_diff_rc = calcRotationEulerZYX(rc.orientation, orientation_ground);

    oss << "groundAnalysis[2]: Caculate each lidar's Euler angles to ground----->  \n" ;
    oss << "Axis continuous angles (fr, fl): [Pitch:" << angles_fr_fl[1] * 180.0 / M_PI << "°, Roll:" << angles_fr_fl[2] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (fr, ground): [Pitch:" << angles_diff_fr[1] * 180.0 / M_PI << "°, Roll:" << angles_diff_fr[2] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (fl, ground): [Pitch:" << angles_diff_fl[1] * 180.0 / M_PI << "°, Roll:" << angles_diff_fl[2] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (ft, ground): [Pitch:" << angles_diff_ft[1] * 180.0 / M_PI << "°, Roll:" << angles_diff_ft[2] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (rt, ground): [Pitch:" << angles_diff_rt[1] * 180.0 / M_PI << "°, Roll:" << angles_diff_rt[2] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (rc, ground): [Pitch:" << angles_diff_rc[1] * 180.0 / M_PI << "°, Roll:" << angles_diff_rc[2] * 180.0 / M_PI << "°] (degrees)\n";

    return oss.str();
}


std::string Check::stereoAnalysis()
{
    std::ostringstream oss;

    // 1. fr-fl, fr-rc, fl-rc计算重叠区域x值
    std::pair<float, float> dis_fl_fr = calcOverlapDis(lidar_front_left_plates.plates, lidar_front_right_plates.plates, "x");
    std::pair<float, float> dis_ft_fr = calcOverlapDis(lidar_front_top_plates.plates, lidar_front_right_plates.plates, "x");
    std::pair<float, float> dis_rc_fr = calcOverlapDis(lidar_rear_center_plates.plates, lidar_front_right_plates.plates, "x");
    std::pair<float, float> dis_rt_fr = calcOverlapDis(lidar_rear_top_plates.plates, lidar_front_right_plates.plates, "x");


    oss << "stereoAnalysis[1]: calcOverlapDis for X----->  \n" ;
    oss << "dis_fl_fr = [" << dis_fl_fr.first << " " << dis_fl_fr.second << "], \n";
    oss << "dis_ft_fr = [" << dis_ft_fr.first << " " << dis_ft_fr.second << "], \n";
    oss << "dis_rc_fr = [" << dis_rc_fr.first << " " << dis_rc_fr.second << "], \n";
    oss << "dis_rt_fr = [" << dis_rt_fr.first << " " << dis_rt_fr.second << "], \n";

    std::pair<float, float> disy_fl_fr = calcOverlapDis(lidar_front_left_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
    std::pair<float, float> disy_ft_fr = calcOverlapDis(lidar_front_top_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
    std::pair<float, float> disy_rc_fr = calcOverlapDis(lidar_rear_center_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");
    std::pair<float, float> disy_rt_fr = calcOverlapDis(lidar_rear_top_plates.plates_for_y, lidar_front_right_plates.plates_for_y, "y");

    oss << "stereoAnalysis[2]: calcOverlapDis for Y----->  \n" ;
    oss << "disy_fl_fr = [" << disy_fl_fr.first << " " << disy_fl_fr.second << "], \n";
    oss << "disy_ft_fr = [" << disy_ft_fr.first << " " << disy_ft_fr.second << "], \n";
    oss << "disy_rc_fr = [" << disy_rc_fr.first << " " << disy_rc_fr.second << "], \n";
    oss << "disy_rt_fr = [" << disy_rt_fr.first << " " << disy_rt_fr.second << "], \n";



    // 2. 计算ft, fl, rt, rc五个雷达的平面法向量到fr的欧拉角
    Eigen::Vector3d angles_fl_fr = calcOverallPlates(lidar_front_left_plates, lidar_front_right_plates);
    Eigen::Vector3d angles_ft_fr = calcOverallPlates(lidar_front_top_plates, lidar_front_right_plates);
    Eigen::Vector3d angles_rt_fr = calcOverallPlates(lidar_rear_top_plates, lidar_front_right_plates);
    Eigen::Vector3d angles_rc_fr = calcOverallPlates(lidar_rear_center_plates, lidar_front_right_plates);


    oss << "Axis continuous angles (fl, fr): [Yaw:"<< angles_fl_fr[0] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (ft, fr): [Yaw:"<< angles_ft_fr[0] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (rt, fr): [Yaw:"<< angles_rt_fr[0] * 180.0 / M_PI << "°] (degrees)\n";
    oss << "Axis continuous angles (rc, fr): [Yaw:"<< angles_rc_fr[0] * 180.0 / M_PI << "°] (degrees)\n";


    return oss.str();
}


void Check::setPlatesData(LidarPlates& lidar_plate, const std::vector<BoxInfo>& boxes, const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_plates)
{    
    lidar_plate.plates.clear();

    if(boxes.size() != cloud_plates.size()) {
        std::cerr << "Error: boxes and cloud_plates size mismatch!" << std::endl;
        return;
    }

    if(boxes.empty())   return;

    lidar_plate.plates.resize(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) 
    {
        // 计算点云的均值作为 position
        const auto& cloud = cloud_plates.at(i);
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        if (!cloud.empty()) {
            for (const auto& pt : cloud.points) {
            mean[0] += pt.x;
            mean[1] += pt.y;
            mean[2] += pt.z;
            }
            mean /= static_cast<float>(cloud.size());
        }
        lidar_plate.plates.at(i).position = mean;
        lidar_plate.plates.at(i).cloud = cloud_plates.at(i);
    }
    // std::cout<<"Set plates data for lidar: " << lidar_plate.lidar_id << ", number of plates: " << lidar_plate.plates.size() << std::endl;
}

void Check::setPlatesData(std::vector<PlateInfo>& plates, const std::vector<BoxInfo>& boxes, const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cloud_plates)
{    
    plates.clear();

    if(boxes.size() != cloud_plates.size()) {
        std::cerr << "Error: boxes and cloud_plates size mismatch!" << std::endl;
        return;
    }

    if(boxes.empty())   return;

    plates.resize(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) 
    {
        // 计算点云的均值作为 position
        const auto& cloud = cloud_plates.at(i);
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        if (!cloud.empty()) {
            for (const auto& pt : cloud.points) {
            mean[0] += pt.x;
            mean[1] += pt.y;
            mean[2] += pt.z;
            }
            mean /= static_cast<float>(cloud.size());
        }
        plates.at(i).position = mean;
        plates.at(i).cloud = cloud_plates.at(i);
    }
    // std::cout<<"Set plates data for lidar: " << lidar_plate.lidar_id << ", number of plates: " << lidar_plate.plates.size() << std::endl;
}

int Check::getPlatesCount(const std::string& lidar_id) const
{
    if (lidar_id == "front_top") {
        return static_cast<int>(lidar_front_top_plates.plates.size());
    } else if (lidar_id == "front_right") {
        return static_cast<int>(lidar_front_right_plates.plates.size());
    } else if (lidar_id == "front_left") {
        return static_cast<int>(lidar_front_left_plates.plates.size());
    } else if (lidar_id == "rear_top") {
        return static_cast<int>(lidar_rear_top_plates.plates.size());
    } else if (lidar_id == "rear_center") {
        return static_cast<int>(lidar_rear_center_plates.plates.size());
    } else {
        std::cerr << "Unknown lidar_id: " << lidar_id << std::endl;
        return -1;
    }
}

int Check::getPlates_for_y_Count(const std::string& lidar_id) const
{
    if (lidar_id == "front_top") {
        return static_cast<int>(lidar_front_top_plates.plates_for_y.size());
    } else if (lidar_id == "front_right") {
        return static_cast<int>(lidar_front_right_plates.plates_for_y.size());
    } else if (lidar_id == "front_left") {
        return static_cast<int>(lidar_front_left_plates.plates_for_y.size());
    } else if (lidar_id == "rear_top") {
        return static_cast<int>(lidar_rear_top_plates.plates_for_y.size());
    } else if (lidar_id == "rear_center") {
        return static_cast<int>(lidar_rear_center_plates.plates_for_y.size());
    } else {
        std::cerr << "Unknown lidar_id: " << lidar_id << std::endl;
        return -1;
    }
}


Check* Check::getInstance()
{
    static Check instance;
    return &instance;
}

void Check::initializeLidarAnalysisMap()
{
    lidar_analysis_map["front_top"] = [this]() { return analyzeFrontTop(true); };
    lidar_analysis_map["front_right"] = [this]() { return analyzeFrontRight(true); };
    lidar_analysis_map["front_left"] = [this]() { return analyzeFrontLeft(true); };
    lidar_analysis_map["rear_top"] = [this]() { return analyzeRearTop(true); };
    lidar_analysis_map["rear_center"] = [this]() { return analyzeRearCenter(true); };
}

PlaneAnalysisResult Check::analyzeLidarByName(const std::string& lidar_name)
{
    PlaneAnalysisResult result;
    
    // 查找对应的分析函数
    auto it = lidar_analysis_map.find(lidar_name);
    if (it == lidar_analysis_map.end()) {
        std::cerr << "Unknown lidar name: " << lidar_name << std::endl;
        return result; // 返回默认的无效结果
    }
    
    // 直接调用对应的分析函数
    result = it->second();
    std::cout << "Analysis completed for lidar: " << lidar_name << std::endl;
    
    return result;
}

PlaneAnalysisResult Check::analyzeFrontTop(const bool& details)
{
    return analyzePlates(lidar_front_top_plates, details);
}

PlaneAnalysisResult Check::analyzeFrontRight(const bool& details)
{
    return analyzePlates(lidar_front_right_plates, details);
}
    

PlaneAnalysisResult Check::analyzeFrontLeft(const bool& details)
{
    return analyzePlates(lidar_front_left_plates, details);
}

PlaneAnalysisResult Check::analyzeRearTop(const bool& details)
{
    return analyzePlates(lidar_rear_top_plates, details);
}

PlaneAnalysisResult Check::analyzeRearCenter(const bool& details)
{
    return analyzePlates(lidar_rear_center_plates, details);
}

PlaneAnalysisResult Check::analyzePlates(const LidarPlates& lidar_plate, const bool& need_details)
{
    PlaneAnalysisResult rel;
    if (lidar_plate.plates.empty()) {
        std::cout << "No plates data for " << lidar_plate.lidar_id<<" lidar" << std::endl;
        return rel;
    }

    std::cout << "Analyzing " << lidar_plate.lidar_id<<" lidar with " << lidar_plate.plates.size() << " plates." << std::endl;

    if(1)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_all;
        for (const auto& plate : lidar_plate.plates) {
            cloud_all += plate.cloud; // 累积所有plate的点云 
        }
        // 使用SVD最小二乘法进行平面拟合（没有噪声的情况下，SVD结果准确且稳定）
        PlaneInfo plane_info = fitPlaneSVD(cloud_all, 0.01);
        rel.orientation[0] = plane_info.coefficients[0];
        rel.orientation[1] = plane_info.coefficients[1];
        rel.orientation[2] = plane_info.coefficients[2];
        rel.describe = "Plane fitting using SVD for all plates:\n";
        rel.describe += "coefficients = [" + std::to_string(plane_info.coefficients[0]) + ", " +
                        std::to_string(plane_info.coefficients[1]) + ", " +
                        std::to_string(plane_info.coefficients[2]) + "], ";
        rel.describe += "centroid = [" + std::to_string(plane_info.centroid[0]) + ", " +
                        std::to_string(plane_info.centroid[1]) + ", " +
                        std::to_string(plane_info.centroid[2]) + "], ";
        rel.describe += "confidence = " + std::to_string(plane_info.confidence) + "\n";
        
        return rel; // 返回结果

    }
    else
    {
        // 对第一个plate进行平面拟合示例
        std::vector<PlaneInfo> plane_infos;
        if (!lidar_plate.plates.empty()) {

            for(auto & plate : lidar_plate.plates) {
                std::cout << "Fitting plane for plate with " << plate.cloud.size() << " points." << std::endl;

                // 使用RANSAC进行平面拟合
                // PlaneInfo plane_info = fitPlaneRANSAC(plate.cloud, 0.01, 100, 0.9);

                // 使用SVD最小二乘法进行平面拟合（没有噪声的情况下，SVD结果准确且稳定）
                PlaneInfo plane_info = fitPlaneSVD(plate.cloud, 0.01);
                

                if (plane_info.is_valid) {
                    plane_infos.push_back(plane_info);

                    // 累积结果到result.describe
                    if(need_details)
                    {
                        std::ostringstream oss;
                        oss << "Plate " << (&plate - &lidar_plate.plates[0]) << ": ";
                        oss << "coefficients = [" << plane_info.coefficients.transpose() << "], ";
                        oss << "centroid = [" << plane_info.centroid.transpose() << "], ";
                        oss << "confidence = " << plane_info.confidence << "\n";
                        rel.describe += oss.str();
                    }

                } else {
                    std::cout << "Plane fitting failed!" << std::endl;
                }
            }

        }
        // 计算平均法向量方向
        if (!plane_infos.empty()) {
            Eigen::Vector4f coeff_sum = Eigen::Vector4f::Zero();
            for (const auto& plane : plane_infos) {
                coeff_sum += plane.coefficients;
            }
            rel.orientation[0] = coeff_sum[0] / static_cast<float>(plane_infos.size());
            rel.orientation[1] = coeff_sum[1] / static_cast<float>(plane_infos.size());
            rel.orientation[2] = coeff_sum[2] / static_cast<float>(plane_infos.size());
            std::ostringstream oss;
            oss << "-----Average plane orientation for " << lidar_plate.lidar_id<<" lidar: ----->   ";
            oss << "x = " << rel.orientation[0] << ", ";
            oss << "y = " << rel.orientation[1] << ", ";
            oss << "z = " << rel.orientation[2] << "\n";
            rel.describe += oss.str();
        }
        return rel;
    }

}

std::pair<float, float> Check::calcOverlapDis(const std::vector<PlateInfo>& plates_1, const std::vector<PlateInfo>& plates_2, const std::string& axis)
{
    if (plates_1.empty() || plates_2.empty()) {
        std::cerr << "Error: One of the LidarPlates is empty!" << std::endl;
        return std::make_pair(0.0f, 0.0f);
    }

    std::vector<std::pair<int, int>> matched_pairs; // 存储匹配的plate对索引
    std::vector<float> axis_differences; // 存储轴向差异

    // 遍历plates_1中的每个plate
    for (size_t i = 0; i < plates_1.size(); ++i) {
        const auto& plate1 = plates_1[i];
        
        // 在plates_2中寻找距离小于0.3的plate
        for (size_t j = 0; j < plates_2.size(); ++j) {
            const auto& plate2 = plates_2[j];
            
            // 计算三维欧式距离
            float distance = (plate1.position - plate2.position).norm();
            
            if (distance < 0.3f) {
                // 找到匹配的plate对，计算z均值差异
                matched_pairs.push_back(std::make_pair(i, j));
                
                // 计算plate1点云的z均值
                float axis_mean_1 = 0.0f;
                if (!plate1.cloud.empty()) {
                    for (const auto& point : plate1.cloud.points) {
                        if("z" == axis || "Z" == axis)
                            axis_mean_1 += point.z;
                        else if("x" == axis || "X" == axis)
                            axis_mean_1 += point.x;
                        else if("y" == axis || "Y" == axis)
                            axis_mean_1 += point.y;
                    }
                    axis_mean_1 /= static_cast<float>(plate1.cloud.size());
                }
                
                // 计算plate2点云的z均值
                float axis_mean_2 = 0.0f;
                if (!plate2.cloud.empty()) {
                    for (const auto& point : plate2.cloud.points) {
                        if("z" == axis || "Z" == axis)
                            axis_mean_2 += point.z;
                        else if("x" == axis || "X" == axis)
                            axis_mean_2 += point.x;
                        else if("y" == axis || "Y" == axis)
                            axis_mean_2 += point.y;
                    }
                    axis_mean_2 /= static_cast<float>(plate2.cloud.size());
                }
                
                // 计算z均值差异
                float z_diff = axis_mean_1 - axis_mean_2;
                axis_differences.push_back(z_diff);
                
               
                // 找到匹配后跳出内层循环，避免一对多匹配
                break;
            }
        }
    }
    
    // 计算统计结果
    if (axis_differences.empty()) {
        std::cout << "No matching plates found !" <<std::endl;
        return std::make_pair(0.0f, 0.0f);
    }
    
    // 计算平均z差异
    float mean_z_diff = 0.0f;
    for (float diff : axis_differences) {
        mean_z_diff += diff;
    }
    mean_z_diff /= static_cast<float>(axis_differences.size());
    
    // 计算标准差
    float std_dev = 0.0f;
    for (float diff : axis_differences) {
        std_dev += (diff - mean_z_diff) * (diff - mean_z_diff);
    }
    std_dev = std::sqrt(std_dev / static_cast<float>(axis_differences.size()));
    
    return std::make_pair(mean_z_diff, std_dev);
}

// 首先寻找相近的plate对，然后计算这些plate对的平面法向量欧拉角差异，最后返回平均欧拉角差异
Eigen::Vector3d Check::calcOverlapPlates(const LidarPlates& plates_1, const LidarPlates& plates_2)
{
    if (plates_1.plates.empty() || plates_2.plates.empty()) {
        std::cerr << "Error: One of the LidarPlates is empty!" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    std::vector<Eigen::Vector3d> matched_planes; // 存储匹配的plate位置

    // 遍历plates_1中的每个plate
    for (size_t i = 0; i < plates_1.plates.size(); ++i) {
        const auto& plate1 = plates_1.plates[i];
        
        // 在plates_2中寻找距离小于0.3的plate
        for (size_t j = 0; j < plates_2.plates.size(); ++j) {
            const auto& plate2 = plates_2.plates[j];
            
            // 计算三维欧式距离
            float distance = (plate1.position - plate2.position).norm();
            
            if (distance < 0.3f) {
                std::cout << "Found matching plates: " << plates_1.lidar_id << "[" << i << "] and " 
                          << plates_2.lidar_id << "[" << j << "], distance: " << distance << std::endl;
                PlaneInfo plane1_info = fitPlaneSVD(plate1.cloud, 0.01);
                PlaneInfo plane2_info = fitPlaneSVD(plate2.cloud, 0.01);
                Eigen::Vector3d euler_angles = calcRotationEulerZYX(plane1_info, plane2_info);
                std::cout << "Matched plates angles: " << i << " and " << j << ": " << euler_angles.transpose()/M_PI*180.0 << std::endl;
                matched_planes.push_back(euler_angles);

                break; // 找到匹配后跳出内层循环，避免一对多匹配
            }
        }
    }
     std::cout << std::endl;
    // 计算统计结果
    if (matched_planes.empty()) {
        std::cout << "No matching plates found between " << plates_1.lidar_id 
                  << " and " << plates_2.lidar_id << std::endl;
        return Eigen::Vector3d::Zero(); 
    }
    // 计算平均旋转角度
    Eigen::Vector3d mean_angles = Eigen::Vector3d::Zero();
    for (const auto& angles : matched_planes) {
        mean_angles += angles;
    }
    mean_angles /= static_cast<double>(matched_planes.size());

    return mean_angles;
}

// 用于plates_1 to plates_2+plates_3 的三个雷达yaw的标定， 计算plates_1与plates_2和plates_3合并后的点云拟合平面欧拉角差异
Eigen::Vector3d Check::calcOverallPlates(const LidarPlates& plates_1, const LidarPlates& plates_2, const LidarPlates& plates_3)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_all_1;
    for (const auto& plate : plates_1.plates) {
        cloud_all_1 += plate.cloud; // 累积所有plate的点云
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_all_target;
    for (const auto& plate : plates_2.plates)       
    {
        if(yaw_cali_done)   //前后侧都有可能检测到平面，在锁定fr fl前，需要使用前侧的plate， 锁定后使用后侧的plate
        {
            if(plate.position[0] < -2.0)    cloud_all_target += plate.cloud; // 累积所有后侧的plate的点云
        }
        else
        {
            if(plate.position[0] > 2.0)    cloud_all_target += plate.cloud; // 累积所有前侧的plate的点云
        }
        
    }
    for(const auto& plate : plates_3.plates) 
    {
        if(yaw_cali_done)
        {
            if(plate.position[0] < -2.0)    cloud_all_target += plate.cloud; // 累积所有后侧的plate的点云
        }
        else
        {
            if(plate.position[0] > 2.0)    cloud_all_target += plate.cloud; // 累积所有前侧的plate的点云
        }
    }

    // 进行平面拟合等操作
    PlaneInfo plane1_info = fitPlaneSVD(cloud_all_1, 0.01);
    PlaneInfo plane2_info = fitPlaneSVD(cloud_all_target, 0.01);
    Eigen::Vector3d euler_angles = calcRotationEulerZYX(plane1_info, plane2_info);

    return euler_angles;
}

// 用于雷达的两两标定，计算两个雷达所有plate的点云拟合平面欧拉角差异（yaw）
Eigen::Vector3d Check::calcOverallPlates(const LidarPlates& plates_1, const LidarPlates& plates_2)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_all_1;
    for (const auto& plate : plates_1.plates) 
    {
            if(yaw_cali_done)   //前后侧都有可能检测到平面，在锁定fr fl前，需要使用前侧的plate， 锁定后使用后侧的plate
            {
                if(plate.position[0] < -2.0)    cloud_all_1 += plate.cloud; // 累积所有后侧的plate的点云
            }
            else
            {
                if(plate.position[0] > 2.0)    cloud_all_1 += plate.cloud; // 累积所有前侧的plate的点云
            }
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_all_2;
    for (const auto& plate : plates_2.plates) 
    {
        if(yaw_cali_done)   //前后侧都有可能检测到平面，在锁定fr fl前，需要使用前侧的plate， 锁定后使用后侧的plate
        {
            if(plate.position[0] < -2.0)    cloud_all_2 += plate.cloud; // 累积所有后侧的plate的点云
        }
        else
        {
            if(plate.position[0] > 2.0)    cloud_all_2 += plate.cloud; // 累积所有前侧的plate的点云
        }
    }

    // 进行平面拟合等操作
    PlaneInfo plane1_info = fitPlaneSVD(cloud_all_1, 0.01);
    PlaneInfo plane2_info = fitPlaneSVD(cloud_all_2, 0.01);
    Eigen::Vector3d euler_angles = calcRotationEulerZYX(plane1_info, plane2_info);

    return euler_angles;
}

PlaneInfo Check::fitPlaneRANSAC(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
                                double distance_threshold,
                                int max_iterations,
                                double probability)
{
    PlaneInfo plane_info;
    
    // 检查输入点云是否有效
    if (cloud.empty()) {
        std::cerr << "Error: Input point cloud is empty!" << std::endl;
        return plane_info;
    }
    
    if (cloud.size() < 3) {
        std::cerr << "Error: Point cloud has less than 3 points, cannot fit a plane!" << std::endl;
        return plane_info;
    }
    
    try {
        // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        // 设置分割参数
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);
        seg.setDistanceThreshold(distance_threshold);
        seg.setProbability(probability);
        
        // 创建输入点云的指针
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
        seg.setInputCloud(cloud_ptr);
        
        // 执行分割
        seg.segment(*inliers, *coefficients);
        
        // 检查是否找到平面
        if (inliers->indices.empty()) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            return plane_info;
        }
        
        // 提取平面系数 (ax + by + cz + d = 0)
        if (coefficients->values.size() != 4) {
            std::cerr << "Error: Invalid plane coefficients!" << std::endl;
            return plane_info;
        }
        
        plane_info.coefficients = Eigen::Vector4f(coefficients->values[0],
                                                 coefficients->values[1], 
                                                 coefficients->values[2],
                                                 coefficients->values[3]);
        
        // 计算点云质心
        Eigen::Vector4f centroid_4f;
        if (pcl::compute3DCentroid(cloud, centroid_4f) != 0) {
            plane_info.centroid = Eigen::Vector3f(centroid_4f[0], centroid_4f[1], centroid_4f[2]);
        } else {
            std::cerr << "Warning: Failed to compute centroid, using zero vector." << std::endl;
            plane_info.centroid = Eigen::Vector3f::Zero();
        }
        
        // 设置其他信息
        plane_info.inlier_count = static_cast<int>(inliers->indices.size());
        plane_info.confidence = static_cast<float>(plane_info.inlier_count) / static_cast<float>(cloud.size());
        plane_info.is_valid = true;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in RANSAC plane fitting: " << e.what() << std::endl;
        plane_info.is_valid = false;
    }
    
    return plane_info;
}

PlaneInfo Check::fitPlaneSVD(const pcl::PointCloud<pcl::PointXYZI>& cloud, 
                             double distance_threshold)
{
    PlaneInfo plane_info;
    
    // 检查输入点云是否有效
    if (cloud.empty()) {
        std::cerr << "Error: Input point cloud is empty!" << std::endl;
        return plane_info;
    }
    
    if (cloud.size() < 3) {
        std::cerr << "Error: Point cloud has less than 3 points, cannot fit a plane!" << std::endl;
        return plane_info;
    }
    
    try {
        // 计算点云质心
        Eigen::Vector4f centroid_4f;
        if (pcl::compute3DCentroid(cloud, centroid_4f) == 0) {
            std::cerr << "Error: Failed to compute centroid!" << std::endl;
            return plane_info;
        }
        
        plane_info.centroid = Eigen::Vector3f(centroid_4f[0], centroid_4f[1], centroid_4f[2]);
        
        // 构建去中心化的点矩阵 (N x 3)
        Eigen::MatrixXf points_centered(cloud.size(), 3);
        for (size_t i = 0; i < cloud.size(); ++i) {
            points_centered(i, 0) = cloud.points[i].x - plane_info.centroid[0];
            points_centered(i, 1) = cloud.points[i].y - plane_info.centroid[1];
            points_centered(i, 2) = cloud.points[i].z - plane_info.centroid[2];
        }
        
        // 计算协方差矩阵的SVD分解
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(points_centered, Eigen::ComputeFullV);
        
        // 最小奇异值对应的特征向量就是平面法向量
        Eigen::Vector3f normal = svd.matrixV().col(2);
        
        // 归一化法向量
        normal.normalize();
        
        // 计算平面方程系数 ax + by + cz + d = 0
        // 由于法向量(a,b,c)已知，质心在平面上，可以计算d
        float d = -(normal[0] * plane_info.centroid[0] + 
                   normal[1] * plane_info.centroid[1] + 
                   normal[2] * plane_info.centroid[2]);
        
        plane_info.coefficients = Eigen::Vector4f(normal[0], normal[1], normal[2], d);
        
        // 计算内点数量和置信度
        int inlier_count = 0;
        for (const auto& point : cloud.points) {
            // 计算点到平面的距离
            float distance = std::abs(normal[0] * point.x + 
                                    normal[1] * point.y + 
                                    normal[2] * point.z + d);
            if (distance <= distance_threshold) {
                inlier_count++;
            }
        }
        
        plane_info.inlier_count = inlier_count;
        plane_info.confidence = static_cast<float>(inlier_count) / static_cast<float>(cloud.size());
        plane_info.is_valid = true;
        
        std::cout << "SVD plane fitting completed. Inliers: " << inlier_count 
                  << "/" << cloud.size() << ", confidence: " << plane_info.confidence << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in SVD plane fitting: " << e.what() << std::endl;
        plane_info.is_valid = false;
    }
    
    return plane_info;
}

// 计算plates的z均值
float Check::calc_z_mean(const std::vector<PlateInfo>& plates) const
{
    if (plates.empty()) return 0.0f;
    float sum = std::accumulate(
        plates.begin(), plates.end(), 0.0f,
        [](float s, const PlateInfo& p) { return s + p.position.z(); }
    );
    return sum / static_cast<float>(plates.size());
}


// 把旋转矩阵 R 分解为 Z-Y-X 欧拉角 (yaw, pitch, roll)
// 输出单位为弧度，范围 yaw ∈ (-π,π], pitch ∈ [-π/2,π/2], roll ∈ (-π,π]
Eigen::Vector3d rot2eulerZYX(const Eigen::Matrix3d& R)
{
    using namespace Eigen;
    Vector3d e;
    // pitch = asin(-R(2,0))
    e[1] = std::asin(-R(2, 0));
    if (std::abs(R(2, 0)) < 0.999999) {         // 非奇异
        e[0] = std::atan2( R(1, 0),  R(0, 0));  // yaw
        e[2] = std::atan2( R(2, 1),  R(2, 2));  // roll
    } else {                                    // 奇异 (gimbal lock)
        e[0] = std::atan2(-R(0, 1), R(1, 1));
        e[2] = 0.0;
    }
    return e;  // [yaw, pitch, roll]
}

Eigen::Vector3d Check::calcRotationEulerZYX(const PlaneInfo& v1, const PlaneInfo& v2)
{
    Eigen::Vector3d vec1(v1.coefficients[0], v1.coefficients[1], v1.coefficients[2]);
    Eigen::Vector3d vec2(v2.coefficients[0], v2.coefficients[1], v2.coefficients[2]);
    return calcRotationEulerZYX(vec1, vec2);
}

Eigen::Vector3d Check::calcRotationEulerZYX(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    using namespace Eigen;

    // 归一化保证单位向量
    Vector3d u1 = v1.normalized();
    Vector3d u2 = v2.normalized();

    const double cos_theta = u1.dot(u2);

    // 如果几乎同向，直接返回 0
    if (std::abs(cos_theta - 1.0) < 1e-6) {
        return Vector3d::Zero();
    }
    // 如果几乎反向，需要选一个垂直轴旋转 π
    if (std::abs(cos_theta + 1.0) < 1e-6) {
        Vector3d axis = (std::abs(u1.x()) < 0.9) ? Vector3d(1,0,0).cross(u1)
                                                 : Vector3d(0,1,0).cross(u1);
        axis.normalize();
        Matrix3d R = Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
        return rot2eulerZYX(R);
    }

    const double theta = std::acos(cos_theta);
    Vector3d k = u1.cross(u2).normalized();   // 转轴
    Matrix3d K = Matrix3d::Zero();
    K(0,1) = -k.z(); K(0,2) =  k.y();
    K(1,0) =  k.z(); K(1,2) = -k.x();
    K(2,0) = -k.y(); K(2,1) =  k.x();        // skew-symmetric

    // Rodrigues 公式：R = I + sinθ·K + (1-cosθ)·K²
    Matrix3d R = Matrix3d::Identity()
               + std::sin(theta) * K
               + (1.0 - cos_theta) * K * K;
    return rot2eulerZYX(R);
}


