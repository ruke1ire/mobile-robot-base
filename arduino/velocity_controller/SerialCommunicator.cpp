#include "Arduino.h"
#include "SerialCommunicator.h"
#include "string.h"
#include "Node.h"

extern double desired_vel_left;
extern double desired_vel_right;
extern double actual_vel_left;
extern double actual_vel_right;

SerialCommunicator::SerialCommunicator(int baudrate)
{
    Serial.begin(baudrate);
    flag = -1;
}

void SerialCommunicator::communicator_loop()
{
    if(Serial.available() > 0){
        char in_byte = Serial.read();
        bool done = tokenize(in_byte);
        if(done == true){
            bool parse_done = parse();
            if(parse_done){
                interpret();
                node.reset();
            }
        }
    }
}

bool SerialCommunicator::tokenize(char in_byte)
{
    static bool buffering = false;
    static unsigned int buffer_index = 0;
    static char buffer[100];

    if(in_byte == '['){
        buffering = true;
        return false;
    }
    else if(in_byte == ']'){
        buffer[buffer_index] = '\0';
        strncpy(token, buffer, buffer_index+1);
        buffering = false;
        buffer_index = 0;
        return true;
    }
    else if(in_byte == ',' || in_byte == ':'){
        buffer[buffer_index] = '\0';
        strncpy(token, buffer, buffer_index+1);
        buffer_index = 0;
        return true;
    }
    else if(in_byte == ' ' || in_byte == '\n' || in_byte == '\t'){
        return false;
    }
    else{
        if(buffering == true){
            buffer[buffer_index] = in_byte;
            buffer_index++;
            return false;
        }
        else{
            return false;
        }
    }
}

bool SerialCommunicator::parse()
{
    static unsigned int var_len = 0;
    if(var_len > 0){
        node.set_variable(token, strlen(token)+1);
        var_len--;
        if(var_len == 0){
            return true;
        }
    }
    if(strcmp(token, GET_VEL_COMMAND) == 0){
        flag = GET_VEL_FLAG;
        node.set_flag(flag);
        var_len = 0;
        return true;
    }
    else if(strcmp(token, GET_POS_COMMAND) == 0){
        flag = GET_POS_FLAG;
        node.set_flag(flag);
        var_len = 0;
        return true;
    }
    else if(strcmp(token, SET_VEL_L_COMMAND) == 0){
        flag = SET_VEL_L_FLAG;
        node.set_flag(flag);
        var_len = 1;
        return false;
    }
    else if(strcmp(token, SET_VEL_R_COMMAND) == 0){
        flag = SET_VEL_R_FLAG;
        node.set_flag(flag);
        var_len = 1;
        return false;
    }
}

void SerialCommunicator::interpret()
{

    if(node.flag == GET_VEL_FLAG){
        char response[100];
        int response_index = 0;
        response[response_index++] = '[';
        for(int i = 0; i < strlen(LEFT_VEL_RESPONSE); i++){
          response[response_index++] = LEFT_VEL_RESPONSE[i];
        }
        response[response_index++] = ':';
        char string_var[100];
        sprintf(string_var, "%d", (int)(actual_vel_left*180/PI*10));
        for(int i = 0; i < strlen(string_var); i++){
            response[response_index++] = string_var[i];
        }
        response[response_index++] = ',';
        for(int i = 0; i < strlen(RIGHT_VEL_RESPONSE); i++){
          response[response_index++] = RIGHT_VEL_RESPONSE[i];
        }
        response[response_index++] = ':';
        string_var[0] = '\0';
        sprintf(string_var, "%d", (int)(actual_vel_right*180/PI*10));
        for(int i = 0; i < strlen(string_var); i++){
            response[response_index++] = string_var[i];
        }
        response[response_index++] = ']';
        response[response_index++] = '\0';
        Serial.println(response);
    }
    else if(node.flag == SET_VEL_L_FLAG){
        desired_vel_left = (double)atoi(node.variable)*PI/180/10;
    }
    else if(node.flag == SET_VEL_R_FLAG){
        desired_vel_right = (double)atoi(node.variable)*PI/180/10;
    }
}
