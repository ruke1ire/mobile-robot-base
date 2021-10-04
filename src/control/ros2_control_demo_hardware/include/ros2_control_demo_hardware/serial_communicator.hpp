#ifndef SerialCommunicator_h
#define SerialCommunicator_h

#define GET_VEL_COMMAND "getvel"
#define GET_POS_COMMAND "getpos"
#define SET_VEL_L_COMMAND "setvell"
#define SET_VEL_R_COMMAND "setvelr"
#define LEFT_VARIABLE 'l'
#define RIGHT_VARIABLE 'r'

class SerialCommunicator
{
    public:
        SerialCommunicator();
        void read_response();
		void get_vel();
    private:
		int serial_port;
        bool tokenize(char in_byte);
        //bool parse();
        //void interpret();
        char token[100];
        unsigned int command_len;
        int flag;
};
#endif
