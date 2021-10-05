// Compile with:
// g++ -pthread -o test_serial main.cpp SerialCommunicator.cpp Node.cpp
//
// C library headers
#include "SerialCommunicator.hpp"
#include <unistd.h> // write(), read(), close()

unsigned int t = 100000;

int main(){
    SerialCommunicator serial_com;
    while(true){
        serial_com.send_command(GETVEL_FLAG, 0);
        serial_com.send_command(SETVELL_FLAG, 2000);
        serial_com.send_command(SETVELR_FLAG, 2000);
        usleep(t);
        serial_com.send_command(GETVEL_FLAG, 0);
        serial_com.send_command(SETVELL_FLAG, 0);
        serial_com.send_command(SETVELR_FLAG, 0);
        usleep(t);
    }
    return 1;
}

