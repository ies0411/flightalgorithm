/**
 * @file tello_tcp.h
 * @author Ethan
 * @brief Tcp/Ip communication with station , Header file
 * @version 0.1
 * @date 2021-12-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __VM_TCP_H__
#define __VM_TCP_H__

/**enum**/
// #include ""
#include "vm_filter.h"
class SendingStatus : protected VmFilter {
   private:
    // char topic_message[256] = {0};
    int sockfd, portno;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    // bool station_complete_flag_ = false;

    bool echoMode = true;

   public:
    // void SetCompleteFlag(bool type);
    // bool GetCompleteFlag(void);
    void SendPacket(uint8_t type);
    bool StationConnect(void);
};

// void SendingStatus::SetCompleteFlag(bool type) {
//     station_complete_flag_ = ((type == true ? true : false));
// }
// bool SendingStatus::GetCompleteFlag(void) {
//     return station_complete_flag_;
// }

/**
 * @brief connecting to socket
 *
 * @return true
 * @return false
 */
// int sockfd, portno, n, choice = 1;
// struct sockaddr_in serv_addr;
// struct hostent *server;
// char buffer[256];
// bool echoMode = false;
// if (argc < 3) {
//     fprintf(stderr, "Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
//     exit(0);
// }
// if (argc > 3)
//     if (strcmp(argv[3], "-e") == 0)
//         echoMode = true;
// portno = atoi(argv[2]);
// sockfd = socket(AF_INET, SOCK_STREAM, 0);
// if (sockfd < 0)
//     error("ERROR opening socket");
// server = gethostbyname(argv[1]);
// bzero((char *)&serv_addr, sizeof(serv_addr));
// serv_addr.sin_family = AF_INET;
// bcopy((char *)server->h_addr,
//       (char *)&serv_addr.sin_addr.s_addr,
//       server->h_length);
// serv_addr.sin_port = htons(portno);
// if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
//     error("ERROR connecting");
// std::cout << "How do you want the client to behave?:\n1. Be able to send messages manually\n2. Subscribe to /client_messages and send whatever's available there\nYour choice:";
// std::cin >> choice;
// while (ros::ok()) {
//     bzero(buffer, 256);
//     if (choice == 1) {
//         printf("Please enter the message: ");
//         fgets(buffer, 255, stdin);
//     } else if (choice == 2) {
//         strcpy(buffer, listener.getMessageValue());
//         loop_rate.sleep();
//     }
//     n = write(sockfd, buffer, strlen(buffer));
//     if (n < 0)
//         error("ERROR writing to socket");
//     if (echoMode) {
//         bzero(buffer, 256);
//         n = read(sockfd, buffer, 255);
//         if (n < 0)
//             error("ERROR reading reply");
//         printf("%s\n", buffer);
//     }
//     ros::spinOnce();
// }
int sockfd, portno, n, choice = 1;
bool SendingStatus::StationConnect(void) {
    char server_addr[] = "10.8.0.6";
    char server_port_addr[] = "9014";

    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    bool echoMode = false;
    // if (argc < 3) {
    //     fprintf(stderr, "Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
    //     exit(0);
    // }
    // if (argc > 3)
    //     if (strcmp(argv[3], "-e") == 0)
    //         echoMode = true;
    ROS_INFO("check1");
    portno = atoi(server_port_addr);
    ROS_INFO("check11");
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        ROS_INFO("ERROR opening socket");
    server = gethostbyname(server_addr);
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    ROS_INFO("check111");
    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_INFO("error");
    }
    ROS_INFO("check1111");
    // error("ERROR connecting");
    // char server_addr[] = "10.8.0.6";
    // char server_port_addr[] = "9010";
    // ROS_INFO("1");
    // portno = atoi(server_port_addr);
    // ROS_INFO("2");
    // sockfd = socket(AF_INET, SOCK_STREAM, 0);
    // if (sockfd < 0) {
    //     ROS_ERROR_STREAM("ERROR opening socket");
    //     return false;
    // }
    // ROS_INFO("3");
    // server = gethostbyname(server_addr);
    // bzero((char *)&serv_addr, sizeof(serv_addr));
    // serv_addr.sin_family = AF_INET;
    // bcopy((char *)server->h_addr,
    //       (char *)&serv_addr.sin_addr.s_addr,
    //       server->h_length);
    // serv_addr.sin_port = htons(portno);
    // ROS_INFO("5");
    // if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    //     ROS_ERROR_STREAM("ERROR connecting");
    //     return false;
    // }
    // ROS_INFO("4");
    // return true;
}

/**
 * @brief send and receive packet protocel
 *
 * @param _TAKEOFF_STATION : requiring takeoff signal to station
 *        _LANDING_STATION : requiring landing signal to station
 *
 * @protocol 'z' : takeoff signal to station , 'x' : landing signal to station
 *           'Z' or 'X' : mission complete signal packet , from station to drone
 */
void SendingStatus::SendPacket(uint8_t type) {
    // ros::Rate loop_rate(10);
    // const int buf_size = 256;

    // while (ros::ok()) {
    // ros::spinOnce();
    // loop_rate.sleep();
    // char buffer[buf_size];
    // choice protocol
    // if (type == _Info_Station_Status_Type::_TAKEOFF_STATION) {
    //     send_command = 'z';
    // } else if (type == _Info_Station_Status_Type::_LANDING_STATION) {
    //     send_command = 'x';
    // }
    // send signal to station
    // char buffer[256] = "position";
    // buffer = "position";
    // char ch = 'a';
    std::string send_data = "position 1 2 3 4, battery : 90";
    // getline(std::cin, packet_data);
    // if (packet_data[0] == '\x03') {
    //     return;
    // }
    // int size_array = packet_data.length();  //len(packet_data);
    // char send_command[size_array];
    // strcpy(send_command, packet_data.c_str());
    // print()
    // std::cout << send_command << std::endl;
    // ROS_INFO("check");
    // packet_data.c_str();
    // packet_data.To
    if (write(sockfd, send_data.c_str(), strlen(send_data.c_str())) < 0) {
        ROS_ERROR_STREAM("ERROR writing to socket");
    } else {
        ROS_INFO_STREAM("sending");
    }
    // take a echo signal from station
    // if (echoMode) {
    //     bzero(buffer, buf_size);
    //     if (read(sockfd, buffer, buf_size) < 0) {
    //         ROS_ERROR_STREAM("ERROR reading reply");
    //     } else {
    // if (buffer[0] == 'Z' || buffer[0] == 'X') {
    //     station_complete_flag_ = true;
    //     return;
    // }
    // }
    // }
    // }
}

#endif
