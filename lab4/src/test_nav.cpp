#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// Definicja krawędzi sześciokąta i prędkości robota
const double SIDE_LENGTH = 1.0;  // Długość boku sześciokąta (w metrach)
const double LINEAR_VELOCITY = 0.2;  // Prędkość liniowa (w metrach na sekundę)
const double ANGULAR_VELOCITY = 3.14 / 18.0;  // Prędkość kątowa (60 stopni na sekundę)
const double ANGULAR_ROTATION = 3.14 / 3.0;

class OdomListener : public rclcpp::Node
{
public:
    OdomListener() : Node("odom_listener")
    {
        // Subskrypcja tematu odometrii
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mobile_base_controller/odom", 10, std::bind(&OdomListener::odomCallback, this, std::placeholders::_1));

        // Publikacja prędkości na temat cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Inicjalizacja zmiennych
        current_side_ = 0;
        move_forward_ = true;

        // Inicjalizacja pozycji początkowej
        prev_position_ = geometry_msgs::msg::Pose();
        prev_orientation_ = 0.0; // Inicjalizacja kąta obrotu
    }

private:
    // Subskrypcja tematu odometrii
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    int current_side_;  // Numer aktualnego boku sześciokąta
    bool move_forward_;  // Flaga wskazująca, czy robot jedzie do przodu, czy skręca
    geometry_msgs::msg::Pose prev_position_;  // Poprzednia pozycja robota
    double prev_orientation_;  // Poprzednia orientacja robota (kąt)

    // Callback, który przetwarza odometrię
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Aktualna pozycja robota
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        // Obliczenie dystansu, który robot przebył od poprzedniego pomiaru
        double dx = current_x - prev_position_.position.x;
        double dy = current_y - prev_position_.position.y;
        double distance_travelled = std::sqrt(dx * dx + dy * dy);

        // Obliczenie kąta obrotu robota z kwaternionu
        double current_orientation = getYaw(msg->pose.pose.orientation);

        // Obliczenie różnicy w kącie obrotu
        double delta_orientation = current_orientation - prev_orientation_;

        // Normalizowanie kąta obrotu w zakresie [-pi, pi]
        if (delta_orientation > 3.14)
        {
            delta_orientation -= 2 * 3.14;
        }
        else if (delta_orientation < -3.14)
        {
            delta_orientation += 2 * 3.14;
        }

        // Sprawdzanie, czy robot osiągnął punkt, w którym ma skręcić
        if (move_forward_)
        {
            if (distance_travelled >= SIDE_LENGTH)
            {
                // Po przejechaniu jednej krawędzi, robot ma wykonać skręt
                move_forward_ = false;
                prev_position_ = msg->pose.pose;
                prev_orientation_ = current_orientation;
            }
        }
        else
        {
            if (std::abs(delta_orientation) >= ANGULAR_ROTATION)
            {
                // Po obróceniu o zadany kąt, robot kontynuuje jazdę do przodu
                move_forward_ = true;
                prev_position_ = msg->pose.pose;
                prev_orientation_ = current_orientation;
                current_side_++;
            }
        }

        // Ruch robota
        moveInHexagon();

        // Debug: Wypisanie kąta obrotu i dystansu
        RCLCPP_INFO(this->get_logger(), "Current Angle: %f, Distance travelled: %f", current_orientation, distance_travelled);
    }

    // Funkcja przekształcająca kwaternion na kąt w radianach (Yaw)
    double getYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        // Wzór na wyliczenie kąta yaw z kwaternionu
        double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // Funkcja sterująca ruchem robota
    void moveInHexagon()
    {
        auto msg = geometry_msgs::msg::Twist();

        if (move_forward_)
        {
            // Poruszamy się do przodu
            msg.linear.x = LINEAR_VELOCITY;
            msg.angular.z = 0.0;
        }
        else
        {
            // Skręcamy o 60 stopni
            msg.linear.x = 0.0;
            msg.angular.z = ANGULAR_VELOCITY;
        }

        // Publikacja prędkości
        cmd_vel_pub_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomListener>());
    rclcpp::shutdown();
    return 0;
}
