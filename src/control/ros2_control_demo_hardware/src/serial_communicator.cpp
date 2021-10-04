#include "ros2_control_demo_hardware/serial_communicator.hpp"
#include "string.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <thread>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

SerialCommunicator::SerialCommunicator()
{
    serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 1) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    struct termios tty;

    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    tty.c_cflag &= ~CRTSCTS;    
    tty.c_cflag |= (CLOCAL | CREAD);                   
    tty.c_iflag |= (IGNPAR | IGNCR);                  
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);          
    tty.c_oflag &= ~OPOST;

    tty.c_cflag &= ~CSIZE;            
    tty.c_cflag |= CS8;              
    tty.c_cflag &= ~PARENB;         
    tty.c_iflag &= ~INPCK;         
    tty.c_iflag &= ~(ICRNL|IGNCR);
    tty.c_cflag &= ~CSTOPB;      
    tty.c_iflag |= INPCK;       
    tty.c_cc[VTIME] = 0.001;  //  1s=10   0.1s=1 *
    tty.c_cc[VMIN] = 0;

    tcsetattr(serial_port,TCSANOW,&tty); 
    cfsetispeed(&tty, B9600);
}

void SerialCommunicator::get_vel(){
	char getvel[] = "[getvel]";
	write(serial_port, getvel, strlen(getvel));
}

void SerialCommunicator::read_response()
{
	while(true){
		char *read_buf;
		int n = read(serial_port, read_buf, 1);
		bool done = tokenize(*read_buf);
		if(done == true){
			break;
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

//bool SerialCommunicator::parse()
//{
//    static unsigned int var_len = 0;
//    if(var_len > 0){
//        node.set_variable(token, strlen(token)+1);
//        var_len--;
//        if(var_len == 0){
//            return true;
//        }
//    }
//    if(strcmp(token, GET_VEL_COMMAND) == 0){
//        flag = GET_VEL_FLAG;
//        node.set_flag(flag);
//        var_len = 0;
//        return true;
//    }
//    else if(strcmp(token, GET_POS_COMMAND) == 0){
//        flag = GET_POS_FLAG;
//        node.set_flag(flag);
//        var_len = 0;
//        return true;
//    }
//    else if(strcmp(token, SET_VEL_L_COMMAND) == 0){
//        flag = SET_VEL_L_FLAG;
//        node.set_flag(flag);
//        var_len = 1;
//        return false;
//    }
//    else if(strcmp(token, SET_VEL_R_COMMAND) == 0){
//        flag = SET_VEL_R_FLAG;
//        node.set_flag(flag);
//        var_len = 1;
//        return false;
//    }
//}
//
//void SerialCommunicator::interpret()
//{
//
//    if(node.flag == GET_VEL_FLAG){
//        char response[100];
//        int response_index = 0;
//        response[response_index++] = '[';
//        response[response_index++] = 'l';
//        response[response_index++] = ':';
//        char string_var[100];
//        sprintf(string_var, "%d", (int)(actual_vel_left*180/PI*10));
//        for(int i = 0; i < strlen(string_var); i++){
//            response[response_index++] = string_var[i];
//        }
//        response[response_index++] = ',';
//        response[response_index++] = 'r';
//        response[response_index++] = ':';
//        string_var[0] = '\0';
//        sprintf(string_var, "%d", (int)(actual_vel_right*180/PI*10));
//        for(int i = 0; i < strlen(string_var); i++){
//            response[response_index++] = string_var[i];
//        }
//        response[response_index++] = ']';
//        response[response_index++] = '\0';
//        Serial.println(response);
//    }
//    else if(node.flag == SET_VEL_L_FLAG){
//        desired_vel_left = (double)atoi(node.variable)*PI/180/10;
//    }
//    else if(node.flag == SET_VEL_R_FLAG){
//        desired_vel_right = (double)atoi(node.variable)*PI/180/10;
//    }
//}
