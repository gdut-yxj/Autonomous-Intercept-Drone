//
// Created by verse on 24-7-31.
//

#ifndef HELLOWORLD_H
#define HELLOWORLD_H

#include <iostream>
#include <rclcpp/node.hpp>

class helloworld : public rclcpp::Node{
public:
    helloworld() : Node("helloworld") {};

    void say_something_node(std::string something)
    {
        std::cout << "say_something_node say:" << something << std::endl;
        
    }
};



#endif //HELLOWORLD_H
