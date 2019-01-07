#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/publisher.h"

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

#define PORT    9999
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


int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "Server");
    ros::NodeHandle n;

    ros::Publisher mag_pose_pub = n.advertise<geometry_msgs::Pose2D>("mag_pose", 100);
    ros::Publisher wheel_velocity_pub = n.advertise<geometry_msgs::Quaternion>("wheel_velocity", 100);
    ros::Publisher wheel_angle_pub = n.advertise<geometry_msgs::Quaternion>("wheel_angle", 100);

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
    socklen_t len;
    geometry_msgs::Pose2D mag_pose;
    int status;
    // quaternion for just storing
    geometry_msgs::Quaternion wheel_velocity_four_wheels;
    geometry_msgs::Quaternion wheel_angle_four_wheels;

    // loop closing
    while(n.ok())
    {
        cout<<"----------------------------------------------"<<endl;

        status = recvfrom(sockfd, (char *)buffer, MAXLINE,
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);

        memcpy(&dataFromVeh, buffer, sizeof(dataFromVeh));


        // print out the message

        printf("HEADER FLAG:    %x\n", dataFromVeh.Header.flag);
        printf("HEADER TYPE:    %x\n", dataFromVeh.Header.type);
        printf("HEADER ID:      %d\n", dataFromVeh.Header.ID);
        printf("HEADER LEN:     %d\n", dataFromVeh.Header.length);
        printf("HEADER SEC:     %d\n", dataFromVeh.Header.sec);
        printf("HEADER MSEC:    %d\n", dataFromVeh.Header.miliSec);

        printf("MAG POSE X:     %f\n", dataFromVeh.mag_pose_x);
        printf("MAG POSE Y:     %f\n", dataFromVeh.mag_pose_y);
        printf("MAG POSE H:     %f\n", dataFromVeh.mag_heading);

        printf("WHEEL VEL FL:   %f\n", dataFromVeh.wheel_velocity_FL);
        printf("WHEEL VEL FR:   %f\n", dataFromVeh.wheel_velocity_FR);
        printf("WHEEL VEL RL:   %f\n", dataFromVeh.wheel_velocity_RL);
        printf("WHEEL VEL RR:   %f\n", dataFromVeh.wheel_velocity_RR);

        printf("WHEEL ANG FL:   %f\n", dataFromVeh.wheel_angle_FL);
        printf("WHEEL ANG FR:   %f\n", dataFromVeh.wheel_angle_FR);
        printf("WHEEL ANG RL:   %f\n", dataFromVeh.wheel_angle_RL);
        printf("WHEEL ANG RR:   %f\n", dataFromVeh.wheel_angle_RR);

        // publish the pose message
        mag_pose.x = dataFromVeh.mag_pose_x;
        mag_pose.y = dataFromVeh.mag_pose_y;
        mag_pose.theta = dataFromVeh.mag_heading;
        mag_pose_pub.publish(mag_pose);

        // publish the wheel messages
        wheel_velocity_four_wheels.x = dataFromVeh.wheel_velocity_FL;
        wheel_velocity_four_wheels.y = dataFromVeh.wheel_velocity_FR;
        wheel_velocity_four_wheels.z = dataFromVeh.wheel_velocity_RL;
        wheel_velocity_four_wheels.w = dataFromVeh.wheel_velocity_RR;

        wheel_angle_four_wheels.x = dataFromVeh.wheel_angle_FL;
        wheel_angle_four_wheels.y = dataFromVeh.wheel_angle_FR;
        wheel_angle_four_wheels.z = dataFromVeh.wheel_angle_RL;
        wheel_angle_four_wheels.w = dataFromVeh.wheel_angle_RR;

        wheel_velocity_pub.publish(wheel_velocity_four_wheels);
        wheel_angle_pub.publish(wheel_angle_four_wheels);


    }


    return 0;
}
