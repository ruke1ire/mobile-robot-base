#include "ros2_control_demo_hardware/Node.hpp"

Node::Node(){
}

void Node::set_variable(int var){
    variable = var;
}

void Node::set_flag(int _flag){
    flag = _flag;
}
