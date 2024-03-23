#include <rclcpp/rclcpp.hpp>
#include "can_control_actuator/common/tools/lp_filter.h"

class LowPassFilterTester : public rclcpp::Node
{
public:
    LowPassFilterTester() : Node("low_pass_filter_tester"), filter_(2, this->now())
    {
    }

public:
    void update()
    {
        // Get the current ROS time
        auto now = this->now();

        // Generate some dummy input signal (e.g., sinusoidal)
        double input_signal = sin(now.nanoseconds() / 1e9); // Convert nanoseconds to seconds

        // Input the signal into the filter
        filter_.input(input_signal);

        // Get the filtered output
        double filtered_output = filter_.output();

        // Print the filtered output
        RCLCPP_INFO(get_logger(), "Filtered output: %f", filtered_output);
    }

    LowPassFilter filter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    LowPassFilterTester lowPassFilterTester;
    lowPassFilterTester.update();
    rclcpp::shutdown();
    return 0;
}
