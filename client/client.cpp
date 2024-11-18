#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <termios.h>
#include <errno.h>
#include "common/mavlink.h"

// 포트 번호 및 기타 상수 정의
#define SERVER_PORT 54321
#define BUFFER_SIZE 1024
#define MAX_EVENTS 10
#define TIMEOUT -1

// 소켓 상태를 나타내는 열거형
enum SocketState { IDLE, CONNECTED };

// 함수 프로토타입 선언
int open_serial_port(const std::string& device, int baudRate);
bool configure_serial_port(int fd, int baudRate);
void set_nonblocking(int fd);
int connect_to_server(const std::string& serverIP, int port);
void close_socket(int& sockfd);
void handle_serial_data(int serial_fd, int sockfd, uint8_t& system_id);
void handle_socket_data(int sockfd, int serial_fd, uint8_t system_id);

int main() {
    std::string serialDevice = "/dev/ttyAMA0";
    int baudRate = 57600;
    std::string serverIP = "192.168.219.117";

    int serial_fd = open_serial_port(serialDevice, baudRate);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port" << std::endl;
        return -1;
    }

    int sockfd = connect_to_server(serverIP, SERVER_PORT);
    if (sockfd < 0) {
        std::cerr << "Failed to connect to server" << std::endl;
        close(serial_fd);
        return -1;
    }

    uint8_t system_id = 0; // 드론의 시스템 ID

    std::cout << "Relay client is running..." << std::endl;

    // epoll 인스턴스 생성
    int epoll_fd = epoll_create1(0);
    if (epoll_fd == -1) {
        perror("epoll_create1 failed");
        close(serial_fd);
        close_socket(sockfd);
        return -1;
    }

    struct epoll_event event;
    struct epoll_event events[MAX_EVENTS];

    // 시리얼 포트 등록 (엣지 트리거 모드)
    event.events = EPOLLIN | EPOLLET;
    event.data.fd = serial_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, serial_fd, &event) == -1) {
        perror("epoll_ctl: serial_fd");
        close(serial_fd);
        close_socket(sockfd);
        close(epoll_fd);
        return -1;
    }

    // 소켓 등록 (엣지 트리거 모드)
    event.events = EPOLLIN | EPOLLET;
    event.data.fd = sockfd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sockfd, &event) == -1) {
        perror("epoll_ctl: sockfd");
        close(serial_fd);
        close_socket(sockfd);
        close(epoll_fd);
        return -1;
    }

    while (true) {
        int nfds = epoll_wait(epoll_fd, events, MAX_EVENTS, TIMEOUT);
        if (nfds == -1) {
            perror("epoll_wait failed");
            break;
        }

        for (int n = 0; n < nfds; ++n) {
            int fd = events[n].data.fd;

            if (fd == serial_fd) {
                if (events[n].events & EPOLLIN) {
                    handle_serial_data(serial_fd, sockfd, system_id);
                }
            } else if (fd == sockfd) {
                if (events[n].events & EPOLLIN) {
                    handle_socket_data(sockfd, serial_fd, system_id);
                }
            }
        }
    }

    close(serial_fd);
    close_socket(sockfd);
    close(epoll_fd);

    return 0;
}

// 시리얼 포트를 열고 설정하는 함수
int open_serial_port(const std::string& device, int baudRate) {
    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("Serial port open failed");
        return -1;
    }
    set_nonblocking(fd);

    if (!configure_serial_port(fd, baudRate)) {
        close(fd);
        return -1;
    }

    return fd;
}

// 시리얼 포트 설정 함수
bool configure_serial_port(int fd, int baudRate) {
    struct termios config;
    if (!isatty(fd)) {
        perror("Not a serial port");
        return false;
    }
    if (tcgetattr(fd, &config) < 0) {
        perror("tcgetattr failed");
        return false;
    }

    // 시리얼 포트 설정
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                        ONOCR | OFILL | OPOST);
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    config.c_cc[VMIN]  = 0;
    config.c_cc[VTIME] = 10;

    // Baud rate 설정
    speed_t speed;
    switch (baudRate) {
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        // 필요한 경우 다른 baud rate 추가
        default:
            perror("Unsupported baud rate");
            return false;
    }
    if (cfsetispeed(&config, speed) < 0 || cfsetospeed(&config, speed) < 0) {
        perror("cfsetispeed/cfsetospeed failed");
        return false;
    }

    if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
        perror("tcsetattr failed");
        return false;
    }
    return true;
}

// 파일 디스크립터를 논블로킹 모드로 설정하는 함수
void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl F_GETFL error");
        return;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl F_SETFL O_NONBLOCK error");
    }
}

// 서버에 연결하는 함수
int connect_to_server(const std::string& serverIP, int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return -1;
    }
    set_nonblocking(sockfd);

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    if (inet_pton(AF_INET, serverIP.c_str(), &address.sin_addr) <= 0) {
        perror("Invalid address");
        close(sockfd);
        return -1;
    }

    if (connect(sockfd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        if (errno != EINPROGRESS && errno != EWOULDBLOCK) {
            perror("Connection failed");
            close(sockfd);
            return -1;
        }
    }

    std::cout << "Connected to server" << std::endl;
    return sockfd;
}

// 소켓을 닫는 함수
void close_socket(int& sockfd) {
    if (sockfd >= 0) {
        close(sockfd);
        sockfd = -1;
    }
    std::cout << "Socket closed" << std::endl;
}

// 시리얼 포트에서 데이터를 읽고 서버로 전달하는 함수 (엣지 트리거 모드 대응)
void handle_serial_data(int serial_fd, int sockfd, uint8_t& system_id) {
    uint8_t buffer[BUFFER_SIZE];
    while (true) {
        int len = read(serial_fd, buffer, sizeof(buffer));
        if (len > 0) {
            for (int i = 0; i < len; ++i) {
                mavlink_message_t msg;
                mavlink_status_t status;
                if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &status)) {
                    // 드론의 시스템 ID 설정 (최초 수신 시)
                    if (system_id == 0) {
                        system_id = msg.sysid;
                        std::cout << "Drone System ID set to " << (int)system_id << std::endl;
                    }

                    // 메시지를 직렬화하여 서버로 전송
                    uint8_t sendBuffer[MAVLINK_MAX_PACKET_LEN];
                    uint16_t sendLen = mavlink_msg_to_send_buffer(sendBuffer, &msg);
                    send(sockfd, sendBuffer, sendLen, 0);
                }
            }
        } else if (len == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 더 이상 읽을 데이터 없음
            break;
        } else if (len == 0) {
            std::cerr << "Serial port disconnected" << std::endl;
            break;
        } else {
            perror("read from serial port failed");
            break;
        }
    }
}

// 서버에서 데이터를 읽고 시리얼 포트로 전달하는 함수 (엣지 트리거 모드 대응)
void handle_socket_data(int sockfd, int serial_fd, uint8_t system_id) {
    uint8_t buffer[BUFFER_SIZE];
    while (true) {
        int len = recv(sockfd, buffer, sizeof(buffer), 0);
        if (len > 0) {
            for (int i = 0; i < len; ++i) {
                mavlink_message_t msg;
                mavlink_status_t status;
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    uint8_t target_system = 0;
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_COMMAND_LONG: {
                            mavlink_command_long_t cmd;
                            mavlink_msg_command_long_decode(&msg, &cmd);
                            target_system = cmd.target_system;
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                            mavlink_param_request_list_t req;
                            mavlink_msg_param_request_list_decode(&msg, &req);
                            target_system = req.target_system;
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_SET: {
                            mavlink_param_set_t param_set;
                            mavlink_msg_param_set_decode(&msg, &param_set);
                            target_system = param_set.target_system;
                            break;
                        }
                        // 필요한 경우 다른 메시지 타입 추가
                        default:
                            // target_system이 없는 메시지는 모두 전달
                            target_system = system_id;
                            break;
                    }

                    if (target_system == system_id || target_system == 0) {
                        // 메시지를 시리얼 포트로 전송
                        uint8_t sendBuffer[MAVLINK_MAX_PACKET_LEN];
                        uint16_t sendLen = mavlink_msg_to_send_buffer(sendBuffer, &msg);
                        write(serial_fd, sendBuffer, sendLen);
                    }
                }
            }
        } else if (len == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 더 이상 읽을 데이터 없음
            break;
        } else if (len == 0) {
            std::cerr << "Server disconnected" << std::endl;
            break;
        } else {
            perror("read from server failed");
            break;
        }
    }
}
