#include <rclcpp/rclcpp.hpp>
#include "stdlib.h"
#include <sstream>
#include <string>
#include "motor_controller.h"
#include "zy_motor_controller.h"

int main(int argc, char ** argv)
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    OptionParam option={0,0,0,0,0,0,0};
    std::shared_ptr<zy_motor_controller> motor_controller_ptr = 
           std::make_shared<zy_motor_controller>(1,0,"hcfa_D3E_1602_1A02_ENI.xml",option,node_options);
    motor_controller_ptr->config();
    
    motor_controller_ptr->server_on();
    motor_controller_ptr->sync_control();
    rclcpp::spin(motor_controller_ptr);

    rclcpp::shutdown();

    return 0;
}
