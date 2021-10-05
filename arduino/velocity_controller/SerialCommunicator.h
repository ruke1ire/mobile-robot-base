#ifndef SerialCommunicator_h
#define SerialCommunicator_h

#include "Arduino.h"
#include "Node.h"

#define GET_VEL_COMMAND "getvel"
#define GET_POS_COMMAND "getpos"
#define SET_VEL_L_COMMAND "setvell"
#define SET_VEL_R_COMMAND "setvelr"
#define LEFT_VEL_RESPONSE "lvel"
#define RIGHT_VEL_RESPONSE "rvel"
#define LEFT_POS_RESPONSE "lpos"
#define RIGHT_POS_RESPONSE "rpos"

#define GET_VEL_FLAG 0
#define GET_POS_FLAG 1
#define SET_VEL_L_FLAG 2
#define SET_VEL_R_FLAG 3

class SerialCommunicator
{
    public:
        SerialCommunicator(int baudrate);
        void communicator_loop();
    private:
        bool tokenize(char in_byte);
        bool parse();
        void interpret();
        char token[100];
        unsigned int command_len;
        int flag;
        Node node;
};
#endif
