#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <filesystem>

using namespace std::chrono_literals;

class PCDRotator : public rclcpp::Node {
public:
    PCDRotator() : Node("pcd_rotator") {
        // 声明参数
        this->declare_parameter<std::string>("input_path", "");
        this->declare_parameter<std::string>("output_path", "");
        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("yaw", 0.0);
        this->declare_parameter<bool>("debug",false);

        // 获取参数
        auto input_path = this->get_parameter("input_path").as_string();
        auto output_path = this->get_parameter("output_path").as_string();
        auto roll = this->get_parameter("roll").as_double();
        auto pitch = this->get_parameter("pitch").as_double();
        auto yaw = this->get_parameter("yaw").as_double();
        auto debug = this->get_parameter("debug").as_bool();

        // 校验文件路径
        if (!std::filesystem::exists(input_path)) {
            RCLCPP_ERROR(this->get_logger(), "Input file does not exist: %s", input_path.c_str());
            return;
        }

        // 加载点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", input_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %ld points", cloud->size());

        // 创建旋转矩阵
        Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotation_matrix = (yaw_angle * pitch_angle * roll_angle).matrix();

        // 应用旋转变换
        if(debug)
        {
        for (auto& point : *cloud) {
            Eigen::Vector3d point_vec(point.x, point.y, point.z);
            point_vec = rotation_matrix * point_vec;
            point.x = point_vec.x();
            point.y = point_vec.y();
            point.z = point_vec.z();
            RCLCPP_INFO(this->get_logger(), "Point: (%f, %f, %f) -> (%f, %f, %f)", point.x, point.y, point.z,
                       point_vec.x(), point_vec.y(), point_vec.z());
        }
        }else {
        for (auto& point : *cloud) {
            Eigen::Vector3d point_vec(point.x, point.y, point.z);
            point_vec = rotation_matrix * point_vec;
            point.x = point_vec.x();
            point.y = point_vec.y();
            point.z = point_vec.z();
        }
        }

        // 创建输出目录
        std::filesystem::path output_dir = std::filesystem::path(output_path).parent_path();
        if (!output_dir.empty() && !std::filesystem::exists(output_dir)) {
            std::filesystem::create_directories(output_dir);
        }

        // 保存点云
        if (pcl::io::savePCDFileASCII(output_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file: %s", output_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully processed and saved point cloud to: %s", 
                   output_path.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDRotator>());
    rclcpp::shutdown();
    return 0;
}