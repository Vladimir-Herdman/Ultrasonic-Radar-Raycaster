#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

int main(){
    // Set up port reading from arduino program
    const char* port_name = "/dev/tty.usbmodem101";  // from arduino port?
    int serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port == -1) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting terminal attributes: " << strerror(errno) << std::endl;
        close(serial_port);
        return 1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;

    tcsetattr(serial_port, TCSANOW, &tty);

    // Read in data (initially unparsed)
    char buffer[256];
    std::string data = "";

    while (true){
        memset(buffer, 0, sizeof(buffer));
        int bytes_read = read(serial_port, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0){
            data.append(buffer, bytes_read);
            size_t measurement_delimiter_pos;

            // Parse data for breakpoints in send info
            while ((measurement_delimiter_pos = data.find("|")) != std::string::npos){
                std::string message = data.substr(0, measurement_delimiter_pos);
                data.erase(0, measurement_delimiter_pos + 1);
                std::cout << "Data Received: " << message << std::endl;

                // Break up info into specific data points and store
                size_t data_delimiter_pos = message.find(':');

                if (data_delimiter_pos != std::string::npos){
                    std::string degree = message.substr(0, data_delimiter_pos);
                    std::string distanceCM = message.substr(data_delimiter_pos + 1);
                    std::cout << "Degree: " << degree << "\nDistance: " << distanceCM << std::endl;
                } else {
                    std::cerr << "Invalid message format: " << message << std::endl;
                }
            }
        }
    }

    // Cleanup and close
    close(serial_port);

    return 0;
}