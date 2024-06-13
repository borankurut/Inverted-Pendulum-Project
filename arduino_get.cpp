#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include "vector.h"
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

Vec3* SharedPosition_p;
float* SharedAngle_p;

Vec3 CartInitialPos = Vec3{0, -10, 0};

#define PI 3.14159

std::mutex serialMutex;

void userInputThread(int serialPort) {
    while (true) {
        double userValue;
        std::cout << "Enter the offset value to set (between -20 and 20): ";
        std::cin >> userValue;
		if(userValue > 20.0)
			userValue = 20.0;
		else if(userValue < -20.0)
			userValue = -20.0;

        std::string doubleString = std::to_string(userValue) + "\n";

        std::lock_guard<std::mutex> lock(serialMutex);
        write(serialPort, doubleString.c_str(), doubleString.length());
    }
}

int main() {

    // shared memory part
    key_t key = ftok("shmfile", 65);

    // shmget returns an identifier in shmid
    int shmid = shmget(key, sizeof(Vec3) + sizeof(float), 0666 | IPC_CREAT);

    // shmat to attach to shared memory
    void* SharedMemory = (Vec3*)shmat(shmid, nullptr, 0);

    SharedPosition_p = static_cast<Vec3*>(SharedMemory);
    SharedAngle_p = reinterpret_cast<float*>(static_cast<char*>(SharedMemory) + sizeof(Vec3));  // Offset to get the next location in shared memory

    *SharedPosition_p = CartInitialPos;
    *SharedAngle_p = 0.0;

    double x = 0.0;
    double alpha = 0.0;

    int serialPort = open("/dev/ttyACM0", O_RDWR);  // Replace /dev/ttyUSB0 with your serial port

    if (serialPort < 0) {
        std::cerr << "Error opening serial port!" << std::endl;
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting terminal attributes!" << std::endl;
        close(serialPort);
        return 1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

	//tty code to read the port.
    tty.c_cflag &= ~PARENB;  
    tty.c_cflag &= ~CSTOPB;  
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;    
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;  

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;  
    tty.c_lflag &= ~ECHOE;  
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;  

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  

    tty.c_oflag &= ~OPOST;  
    tty.c_oflag &= ~ONLCR; 

    tcsetattr(serialPort, TCSANOW, &tty);

    char buffer[256];
    std::string data;

    std::thread inputThread(userInputThread, serialPort);

    while (true) {
        int n = read(serialPort, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';  // Null-terminate the buffer
            data += buffer;  // Append to data string

            // check for a newline (end of message).
            size_t pos = data.find('\n');
            if (pos != std::string::npos) {
                std::string line = data.substr(0, pos);
                data.erase(0, pos + 1);  // remove the line

                std::istringstream ss(line);
                std::string token;
                std::vector<double> values;

                while (std::getline(ss, token, ',')) {
                    try {
                        values.push_back(std::stod(token));
                    } catch (const std::invalid_argument& e) {
//                        std::cerr << "Invalid argument: " << token << std::endl;  debug purposes

                    } catch (const std::out_of_range& e) {
//                        std::cerr << "Out of range: " << token << std::endl;  debug purposes

                    }
                }

                if (values.size() == 2) {
                    double value1 = values[0];
                    double value2 = values[1];

                    x = value1;
                    alpha = value2;

                } else {
//                    std::cerr << "Error: received an incomplete or malformed message: " << line << std::endl; debug purposes
                }
            }
        }

        SharedPosition_p->set_x(x * 100);
        *SharedAngle_p = alpha * 180 / PI;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    inputThread.join();

    close(serialPort);
    return 0;
}
