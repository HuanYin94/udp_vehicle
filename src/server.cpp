#include "ros/ros.h"
#include "ros/console.h"

#include "eigen_conversions/eigen_msg.h"

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
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

#define PORT    7000
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

    float speed_agv_vx;
    float speed_agv_vy;
    float angular_agv_gyro;

    /// zhenhua vehicle sent wheel data before 2019.03

//    float wheel_velocity_FL;
//    float wheel_velocity_FR;
//    float wheel_velocity_RL;
//    float wheel_velocity_RR;
//    float wheel_angle_FL;
//    float wheel_angle_FR;
//    float wheel_angle_RL;
//    float wheel_angle_RR;
};

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "Server");
    ros::NodeHandle n;

    ros::Publisher mag_pose_pub = n.advertise<geometry_msgs::PointStamped>("mag_pose", 1);
    ros::Publisher velocity_angular_pub = n.advertise<geometry_msgs::Vector3Stamped>("velocity_angluar", 1);

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
    int status;

    // quaternion for just storing
    geometry_msgs::PointStamped mag_pose;
    geometry_msgs::Vector3Stamped velocity_angular_data;

    mag_pose.header.frame_id = "world";
    velocity_angular_data.header.frame_id = "vehicle";
    velocity_angular_data.header.frame_id = "vehicle";

    // loop closing
    while(n.ok())
    {

        status = recvfrom(sockfd, (char *)buffer, MAXLINE,
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);

        memcpy(&dataFromVeh, buffer, sizeof(dataFromVeh));

        // no print out, it's in record cpp
        cout<<"---------------------"<<endl;
        printf("HEADER ID:      %d\n", dataFromVeh.Header.ID);

        ros::Time timeStamp = ros::Time::now();

        // publish the pose message using pointXYZ
        mag_pose.header.stamp = timeStamp;
        mag_pose.point.x = dataFromVeh.mag_pose_x;
        mag_pose.point.y = dataFromVeh.mag_pose_y;
        mag_pose.point.z = dataFromVeh.mag_heading;
        mag_pose_pub.publish(mag_pose);

        velocity_angular_data.header.stamp = timeStamp;
        velocity_angular_data.vector.x = dataFromVeh.speed_agv_vx;
        velocity_angular_data.vector.y = dataFromVeh.speed_agv_vy;
        velocity_angular_data.vector.z = dataFromVeh.angular_agv_gyro;

        velocity_angular_pub.publish(velocity_angular_data);

    }

    return 0;
}


/* before 2019.05

int main(int argc, char **argv)
{

    // INIT
    ros::init(argc, argv, "Server");
    ros::NodeHandle n;

    ros::Publisher mag_pose_pub = n.advertise<geometry_msgs::PointStamped>("mag_pose", 100);
    ros::Publisher wheel_velocity_pub = n.advertise<geometry_msgs::QuaternionStamped>("wheel_velocity", 100);
    ros::Publisher wheel_angle_pub = n.advertise<geometry_msgs::QuaternionStamped>("wheel_angle", 100);

    // recorder setup
    string rawDataFileName;
    ofstream rawDataStream;
    n.getParam("/server/rawDataFileName", rawDataFileName);
    cout<<rawDataFileName<<endl;
    rawDataStream.open(rawDataFileName, ios::out);

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
    int status;
    // quaternion for just storing
    geometry_msgs::PointStamped mag_pose;
    geometry_msgs::QuaternionStamped wheel_velocity_four_wheels;
    geometry_msgs::QuaternionStamped wheel_angle_four_wheels;

    mag_pose.header.frame_id = "world";
    wheel_velocity_four_wheels.header.frame_id = "wheel";
    wheel_angle_four_wheels.header.frame_id = "wheel";

    // loop closing
    while(n.ok())
    {

        status = recvfrom(sockfd, (char *)buffer, MAXLINE,
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);

        memcpy(&dataFromVeh, buffer, sizeof(dataFromVeh));

        // no print out, it's in record cpp
        cout<<"---------------------"<<endl;
        printf("HEADER ID:      %d\n", dataFromVeh.Header.ID);

        ros::Time timeStamp = ros::Time::now();

        // publish the pose message using pointXYZ
        mag_pose.header.stamp = timeStamp;
        mag_pose.point.x = dataFromVeh.mag_pose_x;
        mag_pose.point.y = dataFromVeh.mag_pose_y;
        mag_pose.point.z = dataFromVeh.mag_heading;
        mag_pose_pub.publish(mag_pose);



        // publish the wheel messages using four elements in quaternion
        wheel_velocity_four_wheels.header.stamp = timeStamp;
        wheel_velocity_four_wheels.quaternion.x = dataFromVeh.wheel_velocity_FL;
        wheel_velocity_four_wheels.quaternion.y = dataFromVeh.wheel_velocity_FR;
        wheel_velocity_four_wheels.quaternion.z = dataFromVeh.wheel_velocity_RL;
        wheel_velocity_four_wheels.quaternion.w = dataFromVeh.wheel_velocity_RR;

        wheel_angle_four_wheels.header.stamp = timeStamp;
        wheel_angle_four_wheels.quaternion.x = dataFromVeh.wheel_angle_FL;
        wheel_angle_four_wheels.quaternion.y = dataFromVeh.wheel_angle_FR;
        wheel_angle_four_wheels.quaternion.z = dataFromVeh.wheel_angle_RL;
        wheel_angle_four_wheels.quaternion.w = dataFromVeh.wheel_angle_RR;

        wheel_velocity_pub.publish(wheel_velocity_four_wheels);
        wheel_angle_pub.publish(wheel_angle_four_wheels);




        // Save

        rawDataStream << dataFromVeh.Header.flag << "  " << dataFromVeh.Header.type << "  "
                         << dataFromVeh.Header.ID << "  " << dataFromVeh.Header.length << "  "
                            << dataFromVeh.Header.sec << "  " << dataFromVeh.Header.miliSec << "  "
                               << dataFromVeh.mag_pose_x << "  " << dataFromVeh.mag_pose_y << "  "
                                  << dataFromVeh.mag_heading << "  " << dataFromVeh.wheel_velocity_FL << "  "
                                     << dataFromVeh.wheel_velocity_FR << "  " << dataFromVeh.wheel_velocity_RL << "  "
                                       << dataFromVeh.wheel_velocity_RR << "  " << dataFromVeh.wheel_angle_FL << "  "
                                          << dataFromVeh.wheel_angle_FR << "  " << dataFromVeh.wheel_angle_RL << "  "
                                             << dataFromVeh.wheel_angle_RR << "  " << timeStamp << endl;


    }

    rawDataStream.close();

    return 0;
}

    */
