#ifndef Node_h
#define Node_h

#include "Arduino.h"

class Node
{
    public:
        Node();
        void set_variable(char var[], int length);
        void set_flag(int _flag);
        void reset();
        char variable[100];
        int flag;
    private:
        unsigned int var_index;
};

#endif

