#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <chrono>
#include <vector>
#include <numeric>

using namespace std::chrono_literals;

class SquareNav : public rclcpp::Node
{
public:

    SquareNav() : Node("square_nav")
    {
        // Deklaracja parametrów
        this->declare_parameter<double>("side_length", 1.0);
        this->declare_parameter<int>("num_laps", 1);
        this->declare_parameter<bool>("clockwise", false);

        // Pobieranie parametrów
        side_length_ = this->get_parameter("side_length").as_double();
        num_laps_ = this->get_parameter("num_laps").as_int();
        clockwise_ = this->get_parameter("clockwise").as_bool();

        // Subskrypcja odometrii
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mobile_base_controller/odom", 10, std::bind(&SquareNav::odomCallback, this, std::placeholders::_1));
        ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth_odom", 10, std::bind(&SquareNav::groundTruthCallback, this, std::placeholders::_1));

        // Publikacja komend prędkości
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Inicjalizacja zmiennych stanu
        current_side_ = 0;
        current_lap_ = 0;
        move_forward_ = true;
        prev_position_ = geometry_msgs::msg::Pose();
        prev_orientation_ = 0.0;
        start_time_ = std::chrono::steady_clock::now();
        last_log_time_ = start_time_;

        RCLCPP_INFO(this->get_logger(), 
            "SquareNav initialized with side_length=%.2f, num_laps=%d, clockwise=%s",
            side_length_, num_laps_, clockwise_ ? "true" : "false");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    double side_length_;
    int num_laps_;
    bool clockwise_;
    int current_side_;
    int current_lap_;
    bool move_forward_;
    geometry_msgs::msg::Pose prev_position_;
    double prev_orientation_;
    double current_x;
    double current_y;
    double truth_current_x;
    double truth_current_y;
    double current_orientation;
    double truth_current_orientation;
    double current_position_error;
    double current_orientation_error;

    std::vector<double> position_errors_;     
    std::vector<double> orientation_errors_;     
    std::vector<double> average_position_errors_; 
    std::vector<double> average_orientation_errors_; 
    std::vector<double> lap_position_errors_;     
    std::vector<double> lap_orientation_errors_; 
    std::vector<double> times_; 


    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_log_time_;

    void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        truth_current_x = msg->pose.pose.position.x;
        truth_current_y = msg->pose.pose.position.y;
        truth_current_orientation = getYaw(msg->pose.pose.orientation);

        current_position_error = std::pow(current_x - truth_current_x, 2) + std::pow(current_y - truth_current_y, 2);
        current_orientation_error = std::pow(current_orientation - truth_current_orientation, 2);

        // Zapisanie błędów do wektorów
        position_errors_.push_back(current_position_error);
        orientation_errors_.push_back(current_orientation_error);
        lap_position_errors_.push_back(current_position_error);
        lap_orientation_errors_.push_back(current_orientation_error);

        // Aktualny czas
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_time = current_time - start_time_;

        // Logowanie co 3 sekundy
        if (current_time - last_log_time_ >= 10s)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Elapsed time: %f s | Position error: %f | Orientation error: %f",
                        elapsed_time.count(), current_position_error, current_orientation_error);

            times_.push_back(elapsed_time.count());
            last_log_time_ = current_time; // Zaktualizuj czas ostatniego logowania
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x = msg->pose.pose.position.x;
        current_y = msg->pose.pose.position.y;

        double dx = current_x - prev_position_.position.x;
        double dy = current_y - prev_position_.position.y;
        double distance_travelled = std::sqrt(dx * dx + dy * dy);

        current_orientation = getYaw(msg->pose.pose.orientation);
        double delta_orientation = current_orientation - prev_orientation_;

        if (delta_orientation > M_PI)
        {
            delta_orientation -= 2 * M_PI;
        }
        else if (delta_orientation < -M_PI)
        {
            delta_orientation += 2 * M_PI;
        }

        if (move_forward_)
        {
            if (distance_travelled >= side_length_)
            {
                move_forward_ = false;
                prev_position_ = msg->pose.pose;
                prev_orientation_ = current_orientation;
            }
        }
        else
        {
            double target_rotation = (clockwise_ ? -1 : 1) * M_PI / 2;
            if (std::abs(delta_orientation) >= std::abs(target_rotation))
            {
                move_forward_ = true;
                prev_position_ = msg->pose.pose;
                prev_orientation_ = current_orientation;
                current_side_++;

                if (current_side_ >= 4)
                {
                    current_side_ = 0;
                    current_lap_++;

                    // Obliczenie średnich błędów
                    double avg_position_error = std::accumulate(lap_position_errors_.begin(), lap_position_errors_.end(), 0.0) / lap_position_errors_.size();
                    double avg_orientation_error = std::accumulate(lap_orientation_errors_.begin(), lap_orientation_errors_.end(), 0.0) / lap_orientation_errors_.size();

                    // Dodanie średnich błędów do wektorów
                    average_position_errors_.push_back(avg_position_error);
                    average_orientation_errors_.push_back(avg_orientation_error);

                    // Logowanie średnich błędów
                    RCLCPP_INFO(this->get_logger(),
                                "Lap %d/%d completed. Avg Position Error: %f | Avg Orientation Error: %f",
                                current_lap_, num_laps_, avg_position_error, avg_orientation_error);

                    // Czyszczenie wektorów błędów dla następnego okrążenia
                    lap_orientation_errors_.clear();
                    lap_orientation_errors_.clear();
                }
            }
        }

        moveRobot();
        if (current_lap_ >= num_laps_)
        {
            stopRobot();
            return;
        }

        
    }

    double getYaw(const geometry_msgs::msg::Quaternion &quat)
    {
        double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void moveRobot()
    {
        auto msg = geometry_msgs::msg::Twist();

        if (move_forward_)
        {
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = (clockwise_ ? -1 : 1) * 0.3;
        }

        cmd_vel_pub_->publish(msg);
    }

    void stopRobot()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        cmd_vel_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "All laps completed. Stopping robot.");
        printSummary();
        rclcpp::shutdown();
    }

    void printSummary()
    {
        std::cout << "\n================= Temporary Errors =================\n";
        std::cout << "   Time (s)    |  Position Error  | Orientation Error\n";
        for (size_t i = 0; i < times_.size(); ++i)
        {
            double &time = times_[i];
            double &pos_err = position_errors_[i];
            double &ori_err = orientation_errors_[i];
            std::cout << std::setw(12) << time << " | "
                      << std::setw(16) << pos_err << " | "
                      << std::setw(18) << ori_err << "\n";
        }

        std::cout << "\n=============== Cumulative Errors =================\n";
        std::cout << "   Lap   | Avg Position Error | Avg Orientation Error\n";
        for (size_t i = 0; i < average_orientation_errors_.size(); ++i)
        {
            double &pos_err = average_position_errors_[i];
            double &ori_err = average_orientation_errors_[i];
            std::cout << std::setw(8) << i + 1 << " | "
                      << std::setw(18) << pos_err << " | "
                      << std::setw(22) << ori_err << "\n";
        }
    }


    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareNav>());
    rclcpp::shutdown();
    return 0;
}
