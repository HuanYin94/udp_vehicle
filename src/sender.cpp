#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <fstream>
#include <vector>
#include <time.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>


#define PORT    8484
#define MAXLINE 1024
#define BYTENUM 30 // 16 + 4*3 + 2
#define SLEEPSEC 1000000 // 10e6 = 1s

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
    float pose_x;
    float pose_y;
    float heading;
    unsigned short int status;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "Sender");
    ros::NodeHandle n;




    int sockfd;
    char buffer[MAXLINE];
    struct sockaddr_in     servaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr("10.12.218.65");





    msg toVeh;

    toVeh.Header.flag = 0xcece;
    toVeh.Header.type = 0x0001;
    toVeh.Header.ID = 0x0000;
    toVeh.Header.length = 16;
    toVeh.Header.sec = 0;
    toVeh.Header.miliSec = 0;

    toVeh.pose_x = 0;
    toVeh.pose_y = 0;
    toVeh.heading = 0;





    int cnt = 0;
    while(true)
    {
        toVeh.Header.ID = cnt + 1;

        memcpy(buffer, &toVeh, sizeof(toVeh));

        sendto(sockfd, (const char *)buffer, sizeof(toVeh),
            MSG_CONFIRM, (const struct sockaddr *) &servaddr,
                sizeof(servaddr));

        usleep(SLEEPSEC);
        cnt ++; // ID ++
    }

    close(sockfd);

    return 0;
}
