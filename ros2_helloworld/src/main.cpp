//
// Created by verse on 24-7-31.
//

#include "helloworld.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto say_hello_world_node = std::make_shared<HelloWorld>();

    rclcpp::spin(say_hello_world_node);

    rclcpp::shutdown();

    
    return 0;
}

