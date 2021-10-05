// Compile with:
// g++ -pthread -o test_serial main.cpp SerialCommunicator.cpp
//
// C library headers
#include "SerialCommunicator.hpp"
#include <unistd.h> // write(), read(), close()

int main(){
    SerialCommunicator serial_com;
    while(true){
        serial_com.send_command(GETVEL_FLAG, 0);
        serial_com.send_command(SETVELL_FLAG, 2000);
        serial_com.send_command(SETVELR_FLAG, 2000);
        sleep(2);
        serial_com.send_command(GETVEL_FLAG, 0);
        serial_com.send_command(SETVELL_FLAG, 0);
        serial_com.send_command(SETVELR_FLAG, 0);
        sleep(1);
        serial_com.send_command(GETVEL_FLAG, -2000);
        serial_com.send_command(SETVELL_FLAG, -2000);
        serial_com.send_command(SETVELR_FLAG, -2000);
        sleep(2);
        serial_com.send_command(GETVEL_FLAG, 0);
        serial_com.send_command(SETVELL_FLAG, 0);
        serial_com.send_command(SETVELR_FLAG, 0);
        sleep(1);
    }
    return 1;
}

