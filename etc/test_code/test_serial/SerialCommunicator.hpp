#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#define START_SYM "["
#define END_SYM "]"
#define GETVEL_COMMAND "getvel"
#define GETPOS_COMMAND "getpos"
#define SETVELL_COMMAND "setvell"
#define SETVELR_COMMAND "setvelr"
#define LVEL_RESPONSE "lvel"
#define RVEL_RESPONSE "rvel"
#define LPOS_RESPONSE "lpos"
#define RPOS_RESPONSE "rpos"

#define GETVEL_FLAG 0
#define GETPOS_FLAG 1
#define SETVELL_FLAG 2
#define SETVELR_FLAG 3
#define LEFTVEL_FLAG 4
#define RIGHTVEL_FLAG 5
#define LEFTPOS_FLAG 6
#define RIGHTPOS_FLAG 7

#include <termios.h> // Contains POSIX terminal control definitions
#include <string>

#include "Node.hpp"
#include <thread>

class SerialCommunicator{
    public:
        SerialCommunicator();
        void send_command(int flag, int variable);
        int left_vel, right_vel, left_pos, right_pos;
    private:
        std::string token;
        Node node;
        void read_response();
        struct termios tty;
        int serial_port;
        bool tokenize(char char_in);
        bool parse();
        void interpret();
        std::thread read_thread;
};

#endif
