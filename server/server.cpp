#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstdint>
#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>
#include "common/mavlink.h"  // MAVLink 헤더 파일 포함
#include <vector>
#include <unordered_map>
#include <map>
#include <chrono>
#include <climits>
#include <csignal>

// 포트 번호는 65535 이하로 수정
#define PORT 54321
#define BUFFER_SIZE 1024
#define TIMEOUT -1
#define MAX_EVENTS 100

// 클라이언트 타입을 나타내는 열거형
enum ClientType { GCS_MASTER, GCS_SLAVE, DRONE };

bool running = true;  // 서버 실행 상태를 나타내는 플래그

// 클라이언트 정보를 저장하는 구조체
struct ClientInfo {
    int sockfd;
    ClientType type;
    uint8_t system_id; // MAVLink 시스템 ID
    mavlink_status_t mav_status; // MAVLink 상태
    std::chrono::steady_clock::time_point connect_time; // 연결 시각
};

// SIGINT 핸들러 함수
void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "SIGINT received. Shutting down server..." << '\n';
        running = false;  // 실행 상태를 false로 설정하여 메인 루프를 종료
    }
}

void shutdown_server(std::unordered_map<int, ClientInfo> &clients, int server_sock, int kq) {
    // 모든 클라이언트 소켓 닫기
    for (auto &pair : clients) close(pair.second.sockfd);
    // 서버 소켓 닫기
    close(server_sock);
    close(kq);
    std::cout << "Server shutdown completed." << '\n';
}

// 소켓을 논블로킹 모드로 설정하는 함수
void set_nonblocking(int sockfd) {
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "fcntl F_GETFL error: " << strerror(errno) << '\n';
        return;
    }
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) == -1) {
        std::cerr << "fcntl F_SETFL O_NONBLOCK error: " << strerror(errno) << '\n';
    }
}

// 소켓을 닫고 kqueue에서 제거하는 함수
void close_and_remove(int kq, int sockfd, std::unordered_map<int, ClientInfo> &clients, int &master_gcs_fd) {
    if (clients[sockfd].type == GCS_MASTER) {
        master_gcs_fd = -1;
        std::cout << "Master GCS disconnected." << '\n';

        // 새로운 마스터 GCS 선출
        ClientInfo* new_master = nullptr;
        std::chrono::steady_clock::time_point earliest_time = std::chrono::steady_clock::time_point::max();

        for (auto &pair : clients) {
            ClientInfo &client = pair.second;
            if (client.type == GCS_SLAVE) {
                // 연결 시각을 비교하여 가장 먼저 연결된 GCS_SLAVE를 찾음
                if (client.connect_time < earliest_time) {
                    earliest_time = client.connect_time;
                    new_master = &client;
                }
            }
        }

        if (new_master != nullptr) {
            new_master->type = GCS_MASTER;
            master_gcs_fd = new_master->sockfd;
            std::cout << "New Master GCS assigned (fd: " << master_gcs_fd << ")" << '\n';
            // 마스터 GCS 변경 알림을 전송할 수 있음
        } else {
            std::cout << "No GCS_SLAVE available to assign as Master." << '\n';
        }
    }

    // 클라이언트 제거 및 소켓 닫기
    struct kevent change;
    EV_SET(&change, sockfd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    kevent(kq, &change, 1, NULL, 0, NULL);
    close(sockfd);
    clients.erase(sockfd);
}

// 서버 소켓을 생성하는 함수
int create_server_socket(int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return -1;
    }
    // 소켓 논블로킹 모드 설정
    set_nonblocking(sockfd);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    // 서버소켓(port, ip) 주소 재사용 설정
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 소켓 주소 바인딩
    if (::bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) { 
        perror("Bind failed");
        close(sockfd);
        return -1;
    }

    // 클라이언트 연결 대기
    if (listen(sockfd, 10) < 0) {
        perror("Listen failed");
        close(sockfd);
        return -1;
    }

    return sockfd;
}

// 슬레이브 GCS에서 허용할 메시지인지 확인하는 함수
bool is_allowed_slave_message(const mavlink_message_t &msg) {
    // 슬레이브 GCS에서 허용할 메시지의 msgid를 지정
    switch (msg.msgid) {
        // 모니터링 메시지들
        case MAVLINK_MSG_ID_HEARTBEAT:
        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_ATTITUDE:
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        case MAVLINK_MSG_ID_VFR_HUD:
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:

        // 웨이포인트 관련 메시지들 허용
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        //case MAVLINK_MSG_ID_MISSION_COUNT:    GCS가 드론에 보낼 미션 항목의 총 개수를 알릴 때
        //case MAVLINK_MSG_ID_MISSION_ITEM:     GCS가 드론으로 각 미션 항목을 전송할 때
        //case MAVLINK_MSG_ID_MISSION_ITEM_INT: GCS가 정수형 좌표를 사용하여 미션 항목을 드론으로 전송할 때
        case MAVLINK_MSG_ID_MISSION_ACK:

        // 파라미터 관련 메시지들 추가
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        case MAVLINK_MSG_ID_PARAM_MAP_RC:
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
            return true;
        default:
            return false;
    }
}

// 메시지를 드론에게 전달하는 함수
void forward_message_to_drone(std::unordered_map<int, ClientInfo> &clients, mavlink_message_t &msg) {
    uint8_t target_system = 0;

    // 메시지의 msgid에 따라 target_system 추출
    switch (msg.msgid) {
        // 명령 메시지
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            target_system = cmd.target_system;
            break;
        }

        // 파라미터 요청 메시지
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            mavlink_param_request_list_t req;
            mavlink_msg_param_request_list_decode(&msg, &req);
            target_system = req.target_system;
            break;
        }

        // 파라미터 읽기 메시지
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req_read;
            mavlink_msg_param_request_read_decode(&msg, &req_read);
            target_system = req_read.target_system;
            break;
        }

        // 파리미터 설정 메시지
        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t param_set;
            mavlink_msg_param_set_decode(&msg, &param_set);
            target_system = param_set.target_system;
            break;
        }

        // 미션 관련 메시지
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            mavlink_mission_request_list_t mission_req_list;
            mavlink_msg_mission_request_list_decode(&msg, &mission_req_list);
            target_system = mission_req_list.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_COUNT: {
            mavlink_mission_count_t mission_count;
            mavlink_msg_mission_count_decode(&msg, &mission_count);
            target_system = mission_count.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST: {
            mavlink_mission_request_t mission_request;
            mavlink_msg_mission_request_decode(&msg, &mission_request);
            target_system = mission_request.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM: {
            mavlink_mission_item_t mission_item;
            mavlink_msg_mission_item_decode(&msg, &mission_item);
            target_system = mission_item.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_ACK: {
            mavlink_mission_ack_t mission_ack;
            mavlink_msg_mission_ack_decode(&msg, &mission_ack);
            target_system = mission_ack.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            mavlink_mission_clear_all_t mission_clear_all;
            mavlink_msg_mission_clear_all_decode(&msg, &mission_clear_all);
            target_system = mission_clear_all.target_system;
            break;
        }

        /*
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
            mavlink_mission_item_reached_t mission_item_reached;
            mavlink_msg_mission_item_reached_decode(&msg, &mission_item_reached);
            target_system = mission_item_reached.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CURRENT: {
            mavlink_mission_current_t mission_current;
            mavlink_msg_mission_current_decode(&msg, &mission_current);
            target_system = mission_current.target_system;
            break;
        }
        */

        case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
            mavlink_mission_set_current_t mission_set_current;
            mavlink_msg_mission_set_current_decode(&msg, &mission_set_current);
            target_system = mission_set_current.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
            mavlink_mission_item_int_t mission_item_int;
            mavlink_msg_mission_item_int_decode(&msg, &mission_item_int);
            target_system = mission_item_int.target_system;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
            mavlink_mission_request_int_t mission_request_int;
            mavlink_msg_mission_request_int_decode(&msg, &mission_request_int);
            target_system = mission_request_int.target_system;
            break;
        }

        // 필요한 경우 다른 메시지 타입 추가
        default:
            // target_system이 없는 메시지는 모든 드론에게 전달
            for (auto &pair : clients) {
                ClientInfo &client = pair.second;
                if (client.type == DRONE) {
                    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
                    send(client.sockfd, buffer, len, 0);
                }
            }
            return;
    }

    if (target_system != 0) {
        // target_system과 일치하는 드론에게만 메시지 전달
        for (auto &pair : clients) {
            ClientInfo &client = pair.second;
            if (client.type == DRONE && client.system_id == target_system) {
                // 직렬화
                uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
                send(client.sockfd, buffer, len, 0);
                break; // 대상 드론을 찾았으므로 루프 종료
            }
        }
    } else {    // 코드 한번더 작성 
        // target_system이 0인 경우 모든 드론에게 전달
        for (auto &pair : clients) {
            ClientInfo &client = pair.second;
            if (client.type == DRONE) {
                // 직렬화
                uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
                send(client.sockfd, buffer, len, 0);
            }
        }
    }
}

// 슬레이브 GCS의 제어 메시지를 처리하는 함수
void handle_slave_gcs_message(std::unordered_map<int, ClientInfo> &clients, ClientInfo &sender, mavlink_message_t &msg) {
    // 슬레이브 GCS의 제어 메시지 처리
    if (is_allowed_slave_message(msg)) {
        // 허용된 메시지는 무시하거나 필요한 경우 처리
        forward_message_to_drone(clients, msg);
        std::cout << "Slave GCS (fd: " << sender.sockfd << ") sent allowed message." << '\n';
    } else {
        // 슬레이브 GCS에 제어 권한이 없다는 응답을 보냄
        mavlink_message_t ack_msg;
        mavlink_msg_command_ack_pack(1, 200, &ack_msg, msg.msgid, MAV_RESULT_DENIED, 0, 0, msg.sysid, msg.compid);

        // MAVLink 메시지 직렬화
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &ack_msg);
        send(sender.sockfd, buffer, len, 0);

        std::cout << "Slave GCS (fd: " << sender.sockfd << ") attempted to send control message. Denied." << std::endl;
    }
}

// 메시지를 라우팅하는 함수
void route_message(int kq, std::unordered_map<int, ClientInfo> &clients, ClientInfo &sender, mavlink_message_t &msg, int master_gcs_fd) {
    if (sender.type == DRONE) {
        // 드론에서 온 메시지를 모든 GCS에게 전달
        for (auto &pair : clients) {
            ClientInfo &client = pair.second;
            if (client.type == GCS_MASTER || client.type == GCS_SLAVE) {
                uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
                send(client.sockfd, buffer, len, 0);
            }
        }
    } else if (sender.type == GCS_MASTER) {
        // 마스터 GCS에서 온 메시지를 대상 드론에게 전달
        forward_message_to_drone(clients, msg);
    } else if (sender.type == GCS_SLAVE) {
        // 슬레이브 GCS에서 온 메시지 처리
        handle_slave_gcs_message(clients, sender, msg);
    }
}

// 클라이언트로부터 데이터를 수신하고 처리하는 함수
void handle_client_data(int kq, std::unordered_map<int, ClientInfo> &clients, int sockfd, int &master_gcs_fd) {
    ClientInfo &client = clients[sockfd];
    uint8_t buffer[BUFFER_SIZE];
    ssize_t bytes_read;
    mavlink_message_t msg;

    // 임시 버퍼를 사용하여 데이터를 저장
    std::vector<uint8_t> temp_buffer;

    // 데이터를 수신할 때마다 임시 버퍼에 저장
    while ((bytes_read = recv(sockfd, buffer, BUFFER_SIZE, 0)) > 0) {
        // 수신한 데이터를 임시 버퍼에 추가
        temp_buffer.insert(temp_buffer.end(), buffer, buffer + bytes_read);

        // 버퍼에 있는 데이터를 하나씩 처리
        size_t i = 0;
        while (i < temp_buffer.size()) {
            if (mavlink_parse_char(MAVLINK_COMM_0, temp_buffer[i], &msg, &client.mav_status)) {
                // 메시지 수신 완료, MAVLink 메시지 완성

                // 클라이언트의 시스템 ID 업데이트 (초기 연결 시)
                if (client.system_id == 0) {
                    client.system_id = msg.sysid;
                    std::cout << "Client (fd: " << sockfd << ") System ID set to " << static_cast<int>(client.system_id) << '\n';

                    // 첫 번째 메시지의 컴포넌트 ID로 클라이언트 타입 결정
                    if (msg.compid == MAV_COMP_ID_AUTOPILOT1) {
                        client.type = DRONE;
                        std::cout << "Client (fd: " << sockfd << ") identified as DRONE." << std::endl;
                    } else {
                        // master GCS 없을 경우
                        if (master_gcs_fd == -1) {
                            client.type = GCS_MASTER;
                            master_gcs_fd = sockfd;
                            std::cout << "Client (fd: " << sockfd << ") assigned as GCS MASTER." << '\n';
                        } else {
                            client.type = GCS_SLAVE;
                            std::cout << "Client (fd: " << sockfd << ") assigned as GCS SLAVE." << '\n';
                        }
                    }
                }

                // 메시지 라우팅
                route_message(kq, clients, client, msg, master_gcs_fd);
            }
            i++;
        }

        // 처리된 데이터는 임시 버퍼에서 제거
        temp_buffer.erase(temp_buffer.begin(), temp_buffer.begin() + i);
    }

    if (bytes_read == 0) {
        // 연결 종료
        std::cout << "Client disconnected (fd: " << sockfd << ")" << '\n';
        close_and_remove(kq, sockfd, clients, master_gcs_fd);
    } else if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("recv failed");
        close_and_remove(kq, sockfd, clients, master_gcs_fd);
    }
}

int main() {
    // SIGINT 핸들러 등록
    std::signal(SIGINT, signal_handler);

    int server_sock = create_server_socket(PORT);
    if (server_sock == -1) return -1;

    std::cout << "Server is listening on port " << PORT << std::endl;

    // kqueue 인스턴스 생성
    int kq = kqueue();
    if (kq == -1) {
        perror("kqueue failed");
        return -1;
    }

    struct kevent change;
    struct kevent events[MAX_EVENTS];

    // 서버 소켓 등록
    EV_SET(&change, server_sock, EVFILT_READ, EV_ADD | EV_ENABLE, 0, 0, NULL);
    if (kevent(kq, &change, 1, NULL, 0, NULL) == -1) {
        perror("kevent: server_sock");
        return -1;
    }

    // 클라이언트 정보 저장용 맵
    std::unordered_map<int, ClientInfo> clients;
    int master_gcs_fd = -1; // 마스터 GCS의 소켓 FD

    while (running) {
        int nfds = kevent(kq, NULL, 0, events, MAX_EVENTS, NULL);
        if (nfds == -1) {
            perror("kevent failed");
            break;
        }

        for (int n = 0; n < nfds; ++n) {
            // 해당 이벤트의 파일디스크립터 가져오기
            int fd = static_cast<int>(events[n].ident);

            if (fd == server_sock) {
                // 새로운 클라이언트 연결 처리
                struct sockaddr_in client_addr;
                socklen_t addr_len = sizeof(struct sockaddr_in);

                // 새로운 클라이언트 accept
                int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);
                if (client_sock == -1) {
                    perror("accept failed");
                    continue;
                }

                // 클라이언트 소켓 논블로킹 모드 설정
                set_nonblocking(client_sock);

                // 새로운 클라이언트를 kqueue에 등록
                EV_SET(&change, client_sock, EVFILT_READ, EV_ADD | EV_ENABLE, 0, 0, NULL);
                if (kevent(kq, &change, 1, NULL, 0, NULL) == -1) {
                    perror("kevent: client_sock");
                    close(client_sock);
                    continue;
                }

                // 클라이언트 정보 추가
                ClientInfo client_info;
                client_info.sockfd = client_sock;
                client_info.type = GCS_SLAVE; // 기본값은 GCS_SLAVE로 설정하고, 첫 번째 메시지에서 타입을 결정
                client_info.system_id = 0;
                client_info.connect_time = std::chrono::steady_clock::now();

                clients[client_sock] = client_info;

                std::cout << "New client connected (fd: " << client_sock << ")" << std::endl;
            } else {
                // 서버소켓이 아닌 클라이언트소켓 이벤트 발생
                // 기존 클라이언트로부터 데이터 수신 및 처리
                if (events[n].filter == EVFILT_READ) {
                    handle_client_data(kq, clients, fd, master_gcs_fd);
                }
                // 다른 이벤트 처리는 필요 시 추가
            }
        }
    }
    // 서버 종료 처리
    shutdown_server(clients, server_sock, kq);
    return 0;
}
