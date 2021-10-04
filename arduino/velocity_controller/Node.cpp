#include "Arduino.h"
#include "Node.h"

Node::Node(){
    reset();
}

void Node::set_variable(char *var, int length){
    strncpy(variable, var, length);
}

void Node::set_flag(int _flag){
    flag = _flag;
}

void Node::reset(){
    flag = -1;
    variable[0] = '\0';
}
