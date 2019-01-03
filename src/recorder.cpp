#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
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

#define VEH_LEN  8.800
#define VEH_WID  2.394
#define TIRE_RD  0.875 // nouse

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

inline float to_radians(float degrees) {
    return degrees * (M_PI / 180.0);
}

float wheel_velocity_on_center(float wheel_velocity, float wheel_angle )
{
    return wheel_velocity*std::cos(to_radians(wheel_angle));
}

float wheel_angular_on_center(float wheel_velocity_on_C, float wheel_angle, bool is_out )
{
    float turn_radius;
    if(is_out)
    {
        turn_radius = 0.5*(-VEH_WID + VEH_LEN/std::cos(std::abs(to_radians(wheel_angle))));
    }
    else
    {
        turn_radius = 0.5*(VEH_WID + VEH_LEN/std::cos(std::abs(to_radians(wheel_angle))));
    }

    return wheel_velocity_on_C / turn_radius;

}


int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "Recorder");
    ros::NodeHandle n;

    // recorder setup
    string rawDataFileName;
    string pubDataFileName;
    ofstream rawDataStream;
    ofstream pubDataStream;
    n.getParam("rawDataFileName", rawDataFileName);
    n.getParam("pubDataFileName", pubDataFileName);
    rawDataStream.open(rawDataFileName);
    pubDataStream.open(pubDataFileName);

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
    geometry_msgs::Vector3 wheel_odom;
    float v_on_center_FL, v_on_center_FR, v_on_center_RL, v_on_center_RR,
            w_on_center_FL, w_on_center_FR, w_on_center_RL, w_on_center_RR;

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

        // Save
        rawDataStream << dataFromVeh.Header.flag << "  " << dataFromVeh.Header.type << "  "
                         << dataFromVeh.Header.ID << "  " << dataFromVeh.Header.length << "  "
                            << dataFromVeh.Header.sec << "  " << dataFromVeh.Header.miliSec << "  "
                               << dataFromVeh.mag_pose_x << "  " << dataFromVeh.mag_pose_y << "  "
                                  << dataFromVeh.mag_heading << "  " << dataFromVeh.wheel_velocity_FL << "  "
                                     << dataFromVeh.wheel_velocity_FR << "  " << dataFromVeh.wheel_velocity_RL << "  "
                                       << dataFromVeh.wheel_velocity_RR << "  " << dataFromVeh.wheel_angle_FL << "  "
                                          << dataFromVeh.wheel_angle_FR << "  " << dataFromVeh.wheel_angle_RL << "  "
                                             << dataFromVeh.wheel_angle_RR << endl;

        // publish the pose message
        mag_pose.x = dataFromVeh.mag_pose_x;
        mag_pose.y = dataFromVeh.mag_pose_y;
        mag_pose.theta = dataFromVeh.mag_heading;

        // compute and publish wheel odometry message
        v_on_center_FL = wheel_velocity_on_center(dataFromVeh.wheel_velocity_FL, dataFromVeh.wheel_angle_FL);
        v_on_center_FR = wheel_velocity_on_center(dataFromVeh.wheel_velocity_FR, dataFromVeh.wheel_angle_FR);
        v_on_center_RL = wheel_velocity_on_center(dataFromVeh.wheel_velocity_RL, dataFromVeh.wheel_angle_RL);
        v_on_center_RR = wheel_velocity_on_center(dataFromVeh.wheel_velocity_RR, dataFromVeh.wheel_angle_RR);

        w_on_center_FL = wheel_angular_on_center(dataFromVeh.wheel_velocity_FL, dataFromVeh.wheel_angle_FL, true);
        w_on_center_FR = wheel_angular_on_center(dataFromVeh.wheel_velocity_FR, dataFromVeh.wheel_angle_FR, false);
        w_on_center_RL = wheel_angular_on_center(dataFromVeh.wheel_velocity_RL, dataFromVeh.wheel_angle_RL, true);
        w_on_center_RR = wheel_angular_on_center(dataFromVeh.wheel_velocity_RR, dataFromVeh.wheel_angle_RR, false);

        // velocity in x, angular in y
        wheel_odom.x = (v_on_center_FL + v_on_center_FR + v_on_center_RL + v_on_center_RR) / 4;
        wheel_odom.y = (w_on_center_FL + w_on_center_FR + w_on_center_RL + w_on_center_RR) / 4;
        wheel_odom.z = 0;

        pubDataStream << mag_pose.x << "  " << mag_pose.y << "  " << mag_pose.theta << "  "
                      << wheel_odom.x << "  " << wheel_odom.y << "  " << wheel_odom.z << endl;

    }

    rawDataStream.close();
    pubDataStream.close();
    return 0;
}
