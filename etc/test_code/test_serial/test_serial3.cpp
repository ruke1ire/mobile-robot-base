// Compile with:
// g++ -pthread -o test_serial3 test_serial3.cpp
//
// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <thread>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

void read_serial(int serial_port){
    while(1){
        char *read_buf;
        int n = read(serial_port, read_buf, 1);
        std::cout << *read_buf;
    }
}

int main(){
    int serial_port = open("/dev/ttyACM0", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Create new termios struct, we call it 'tty' for convention
// No need for "= {0}" at the end as we'll immediately write the existing
// config to this struct
    struct termios tty;

// Read in existing settings, and handle any error
// NOTE: This is important! POSIX states that the struct passed to tcsetattr()
// must have been initialized with a call to tcgetattr() overwise behaviour
// is undefined
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

    std::thread t1(read_serial,serial_port);

    while(1){
        char getvel[] = "[getvel]";
        write(serial_port, getvel, strlen(getvel));
        sleep(1);
    }
}
