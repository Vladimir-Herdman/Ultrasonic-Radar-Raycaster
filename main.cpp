#include <opencv2/opencv.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <map>

// Global Constants section
const int width = 240;  // Go out five rings to measure 50 cm and 10 extra as padding
const int height = 140;  // Go out radius of largest ring (50) and 20 more for top/bottom padding/text
const cv::Size size(width, height);
const int scale = 3;
const cv::Point circle_center(width/2, height - 20);

const cv::Scalar green(0, 180, 0);
const cv::Scalar red(255, 8, 0);
const cv::Scalar background(30, 30, 30);

const int fontFace = cv::FONT_HERSHEY_PLAIN;
const double fontScale = 0.5;
const cv::Point angle_display(5, height-5);
const cv::Point distance_display(width/2-20, height-5);

// Helper functions used throughout
void drawLineAtAngle(cv::Mat& frame, cv::Point start, int angle, int length, cv::Scalar color) {
    const double angle_radians = (angle * (M_PI / 180));

    // Section for line
    int end_x = start.x + cos(angle_radians) * length;
    int end_y = start.y - sin(angle_radians) * length;
    cv::line(frame, start, cv::Point(end_x, end_y), color);

    // Section for text
    end_x = start.x + cos(angle_radians) * (length+3);
    end_y = start.y - sin(angle_radians) * (length+3);
    if (angle >= 90) {end_x -= 8;}
    cv::putText(frame, std::to_string(angle), cv::Point(end_x, end_y), fontFace, fontScale, green);
}

void drawRadar(cv::Mat& frame){
    // Base frames
    frame = cv::Mat::zeros(size, CV_8UC3);
    frame.setTo(background);
    cv::Mat larger_frame = cv::Mat::zeros(size*scale, CV_8UC3);

    // Circles
    cv::circle(frame, circle_center, 3, green, -1);
    for (int radius = 1; radius < 6; radius++){
        cv::circle(frame, circle_center, radius*20, green);
    }
    
    // Angle lines
    for (int angle = 1; angle < 6; angle++) {
        drawLineAtAngle(frame, circle_center, angle*30, 104, green);
    }
    
    // Bottom info section
    cv::line(frame, cv::Point(0, height-21), cv::Point(width, height-21), green);
    cv::rectangle(frame, cv::Point(0, height-20), cv::Point(width, height), cv::Scalar(15, 15, 15), -1);
    cv::putText(frame, "Degree: ", angle_display, fontFace, 0.8, green);
    cv::putText(frame, "Distance: ", distance_display, fontFace, 0.8, green);

    // upscale radar
    cv::resize(frame, larger_frame, larger_frame.size(), 0, 0, cv::INTER_CUBIC);
    cv::imshow("Larger Radar (upscaled)", larger_frame);
    cv::waitKey(0);
}

int main(){
    // Map for data later used to build raycasting area
    std::map<int, int> arduino_measurements;

    // opencv for displaying radar frame
    cv::Mat radar;
    drawRadar(radar);

    // Set up port reading from arduino program
    const char* port_name = "/dev/tty.usbmodem101";  // from arduino port?
    int serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port == -1) {
        std::cerr << "Error opening serial port (connect it?): " << strerror(errno) << std::endl;
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

    // Read in data section, meat of code to update drawings and set up
    // map for later raycast "level"
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
                    int degree = std::stoi(message.substr(0, data_delimiter_pos));
                    // Here, we take "float" string data and convert to
                    // int to later simplify mapping and drawing
                    int distanceCM = std::stoi(message.substr(data_delimiter_pos + 1));

                    // Update radar screen


                    // store in measurements if small enough
                    if (distanceCM < 50 && distanceCM > 1 && arduino_measurements.count(degree) == 0){
                        arduino_measurements[degree] = distanceCM;
                    }
                } else {
                    std::cerr << "Invalid message format (breakpoints?): " << message << std::endl;
                }
            }
        }
    }

    // Cleanup and close
    close(serial_port);

    return 0;
}