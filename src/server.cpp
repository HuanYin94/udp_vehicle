#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>

#include <fstream>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT     9999
#define MAXLINE 1024
#define BYTENUM 60 // 16 for header only

using namespace std;

struct header
{
    unsigned short int flag;
    unsigned short int type;
    unsigned short int ID;
    unsigned short int length;
    unsigned int sec;
    unsigned int miliSec;
};

struct msg
{
    struct header Header;
    float mag_pose_x;
    float mag_pose_y;
    float mag_heading;
    float wheel_velocity_FL;
    float wheel_velocity_FR;
    float wheel_velocity_RL;
    float wheel_velocity_RR;
    float wheel_angle_FL;
    float wheel_angle_FR;
    float wheel_angle_RL;
    float wheel_angle_RR;
};

// no object coding

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Server");
    ros::NodeHandle n;

    // SOCKET
    int sockfd;
    char buffer[MAXLINE];
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
            sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }


    // MSG
    msg dataFromVeh;
    int status;
    socklen_t len;

    while(n.ok())
    {
        cout<<"----------------------------------------------"<<endl;

        status = recvfrom(sockfd, (char *)buffer, MAXLINE,
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);

        memcpy(&dataFromVeh, buffer, BYTENUM);

    }


    return 0;
}
