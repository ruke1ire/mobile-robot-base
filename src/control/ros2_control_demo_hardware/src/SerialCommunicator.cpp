#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <cstring>
#include <string>
#include <iostream>

#include "ros2_control_demo_hardware/SerialCommunicator.hpp"
#include "ros2_control_demo_hardware/Node.hpp"

SerialCommunicator::SerialCommunicator(){
    serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

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
    tty.c_cc[VTIME] = 10.0;
    tty.c_cc[VMIN] = 1;

    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);
    tcsetattr(serial_port,TCSANOW,&tty); 

    left_vel = 0;
    right_vel = 0;
    left_pos = 0;
    right_pos = 0;
    sleep(3);
}
bool SerialCommunicator::tokenize(char char_in){
    bool done = false;

    //std::cout << char_in;
    if(char_in == ']' || char_in == ':' || char_in == ','){
        done = true; 
    }
    else if(char_in == ' ' || char_in == '\n' || char_in == '\t' || char_in == '['){
    }
    else if(isprint(char_in) == 0){}
    else{
        token += char_in;
    }
    return done;
}

bool SerialCommunicator::parse(){
    static bool done = false;
    int flag = -1;

    done = false;
    if(token.compare(std::string(LVEL_RESPONSE))==0){
        flag = LEFTVEL_FLAG;
    }
    else if(token.compare(std::string(RVEL_RESPONSE))==0){
        flag = RIGHTVEL_FLAG;
    }
    else if(token.compare(std::string(LPOS_RESPONSE))==0){
        flag = LEFTPOS_FLAG;
    }
    else if(token.compare(std::string(RPOS_RESPONSE))==0){
        flag = RIGHTPOS_FLAG;
    }
    else{
        done = true;
        node.set_variable(std::stoi(token));
    }

    if(flag != -1){
        node.set_flag(flag);
    }

    return done;
}

void SerialCommunicator::send_command(int flag, int variable){
    char command[100];
    char variable_str[100];
    bool isvar = false;

    if(flag == GETVEL_FLAG){
        strcpy(command, GETVEL_COMMAND);
    }
    else if(flag == GETPOS_FLAG){
        strcpy(command, GETPOS_COMMAND);
    }
    else if(flag == SETVELL_FLAG){
        strcpy(command, SETVELL_COMMAND);
        sprintf(variable_str, "%d", variable);
        isvar = true;
    }
    else if(flag == SETVELR_FLAG){
        strcpy(command, SETVELR_COMMAND);
        sprintf(variable_str, "%d", variable);
        isvar = true;
    }

    char command_full[300];
    if(isvar){
    sprintf(command_full,"%s%s:%s%s",START_SYM,command,variable_str,END_SYM);
    }
    else{
    sprintf(command_full,"%s%s%s",START_SYM,command,END_SYM);
    }
    //printf("Sending %s\n", command_full);
    write(serial_port, command_full, strlen(command_full));

    if(flag == GETVEL_FLAG || flag == GETPOS_FLAG){
        int count = 0;
        while(true){
            char char_in[1];
            read(serial_port, char_in, 1);
            //std::cout << char_in[0];
            bool done_tokenize = tokenize(char_in[0]);
            if(done_tokenize){
                //std::cout << token << std::endl;
                bool done_parse = parse();
                if(done_parse){
                    interpret();
                    //std::cout << node.flag << "\t" << node.variable << "\t" << std::endl;
                    count++;
                    if(count == 2){
                        token.clear();
                        break;
                    }
                }
                token.clear();
            }
        }
    }

    return;
}


void SerialCommunicator::interpret(){
    if(node.flag == LEFTVEL_FLAG){
        left_vel = node.variable;
    }
    else if(node.flag == RIGHTVEL_FLAG){
        right_vel = node.variable;
    }
    else if(node.flag == LEFTPOS_FLAG){
        left_pos = node.variable;
    }
    else if(node.flag == RIGHTPOS_FLAG){
        right_pos = node.variable;
    }
    return;
}
