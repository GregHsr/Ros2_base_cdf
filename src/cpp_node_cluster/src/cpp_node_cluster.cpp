#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <vector>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>

class Cpp_Node_Cluster : public rclcpp::Node
{
public:
    Cpp_Node_Cluster() : Node("cpp_node_cluster")
    {
        // Souscription aux données LiDAR
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObjectDetectionNode::laser_callback, this, std::placeholders::_1));

        // Publisher pour les positions d'objets détectés
        object_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("detected_objects", 10);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std_msgs::msg::Float32MultiArray object_centers;
        
        float distance_threshold = 0.5; // Seuil de distance pour séparer les objets
        bool in_object = false;
        std::vector<Eigen::Vector2f> object_points;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range <= msg->range_max) {
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);

                if (in_object && !object_points.empty() && 
                    (object_points.back() - Eigen::Vector2f(x, y)).norm() > distance_threshold) {
                    calculate_circle_center(object_centers, object_points);
                    object_points.clear();
                }
                
                object_points.emplace_back(x, y);
                in_object = true;
            } else if (in_object) {
                calculate_circle_center(object_centers, object_points);
                object_points.clear();
                in_object = false;
            }
        }
        
        if (!object_points.empty()) {
            calculate_circle_center(object_centers, object_points);
        }

        object_pub_->publish(object_centers);
    }

    void calculate_circle_center(std_msgs::msg::Float32MultiArray &object_centers,
                                 const std::vector<Eigen::Vector2f> &points) {
        if (points.size() < 3) return;

        Eigen::MatrixXf A(points.size(), 3);
        Eigen::VectorXf b(points.size());
        
        for (size_t i = 0; i < points.size(); ++i) {
            A(i, 0) = -2 * points[i].x();
            A(i, 1) = -2 * points[i].y();
            A(i, 2) = 1;
            b(i) = -std::pow(points[i].x(), 2) - std::pow(points[i].y(), 2);
        }

        Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
        
        float center_x = x(0);
        float center_y = x(1);
        
        object_centers.data.push_back(center_x);
        object_centers.data.push_back(center_y);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr object_pub_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}

