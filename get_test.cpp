#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>
#include <string>
#include <vector>

int main() {
  int serialPort = open("/dev/ttyUSB0", O_RDWR);  

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

  while (true) {
    int n = read(serialPort, buffer, sizeof(buffer) - 1);
    if (n > 0) {
      buffer[n] = '\0';  
      data += buffer;  


      size_t pos = data.find('\n');
      if (pos != std::string::npos) {
        std::string line = data.substr(0, pos);
        data.erase(0, pos + 1);  

        std::istringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
          try {
            values.push_back(std::stod(token));
          } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: " << token << std::endl;
          } catch (const std::out_of_range& e) {
            std::cerr << "Out of range: " << token << std::endl;
          }
        }

        if (values.size() == 2) {
          double value1 = values[0];
          double value2 = values[1];
          std::cout << "Received values: " << value1 << ", " << value2 << std::endl;
        } else {
          std::cerr << "Error: received an incomplete or malformed message: " << line << std::endl;
        }
      }
    }
    usleep(30000);  
  }

  close(serialPort);
  return 0;
}
