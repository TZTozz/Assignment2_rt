#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlesim_custom_msgs/msg/distance_angle.hpp"
#include <cmath>
#include <vector>
#include <limits>

using std::placeholders::_1;

class Distance : public rclcpp::Node
{
public:
    Distance() : Node("distance_node")
    {
        threshold_ = 1.0;
        alreadyStopped = false;
        angle_min_ = 0.0;
        angle_increment_ = 0.0;


        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Distance::topic_callback, this, _1));

        publisherDistance_ = this->create_publisher<turtlesim_custom_msgs::msg::DistanceAngle>("distance", 10);
        publisherStop_ = this->create_publisher<std_msgs::msg::Bool>("stop", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Distance::timer_callback, this));
            
        msg_stop.data = false;
    }

private:

    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        ranges_ = msg->ranges;
        angle_min_ = msg->angle_min;
        angle_increment_ = msg->angle_increment;
    }


    void timer_callback()
    {
        //In case of no data
        if (ranges_.empty()) return;

        int index = -1;
        float min_range = std::numeric_limits<float>::infinity();

        //Search for the mininum distance
        for (size_t i = 0; i < ranges_.size(); i++)
        {
            float range = ranges_[i];

            if (std::isinf(range) || std::isnan(range)) {
                continue;
            }

            if (range < min_range) {
                min_range = range;
                index = i;
            }
        }

        
        if (index != -1) {

            //Compute the angle
            float angle_obstacle = angle_min_ + (index * angle_increment_);
            
            msg_distance.angle = angle_obstacle * (180.0 / M_PI);
            msg_distance.distance = min_range;
            
            RCLCPP_INFO(this->get_logger(), "Ostacolo a %.2f m, Angolo: %.2f deg", min_range, msg_distance.angle);
        } else {
            msg_distance.distance = -1.0;
            msg_distance.angle = 0.0;
        }

        msg_distance.threshold = threshold_; 

        publisherDistance_->publish(msg_distance);


        //Stop logic
        if (min_range < threshold_ && index != -1)
        {
            if (!alreadyStopped) {
                msg_stop.data = true;
                publisherStop_->publish(msg_stop);
                RCLCPP_WARN(this->get_logger(), "STOP! Ostacolo troppo vicino: %.2fm", min_range);
                alreadyStopped = true;
            }
        }
        else
        {
            if (alreadyStopped)
            {
                msg_stop.data = false;
                publisherStop_->publish(msg_stop);
                RCLCPP_INFO(this->get_logger(), "Via libera. Ripartenza.");
                alreadyStopped = false;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_; // Nome corretto
    rclcpp::Publisher<turtlesim_custom_msgs::msg::DistanceAngle>::SharedPtr publisherDistance_; // Tipo corretto
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisherStop_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> ranges_;
    turtlesim_custom_msgs::msg::DistanceAngle msg_distance;
    std_msgs::msg::Bool msg_stop;
    
    float angle_min_;
    float angle_increment_;
    float threshold_;
    bool alreadyStopped;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance>());
    rclcpp::shutdown();
    return 0;
}