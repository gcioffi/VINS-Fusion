/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
double last_vio_t = -1;
std::queue<geometry_msgs::PoseStampedPtr> globalPosQueue;
std::mutex m_buf;

/*void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}*/

void readParameters(std::string config_file, std::string& topic, Eigen::Matrix4d& T_BP)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["global_position_topic"] >> topic;

    cv::Mat cv_T;
    fsSettings["body_T_p"] >> cv_T;
    cv::cv2eigen(cv_T, T_BP);
}

void leica_callback(const geometry_msgs::PoseStampedPtr &global_pos_msg)
{
    //printf("gps_callback! \n");

    // Debug
    //std::cout << "Im here 1\n";
    // end debug

    m_buf.lock();
    globalPosQueue.push(global_pos_msg);
    m_buf.unlock();
}

void vicon_callback(const geometry_msgs::TransformStampedPtr &global_pos_msg)
{
    //printf("gps_callback! \n");
    geometry_msgs::PoseStampedPtr msg;
    msg->header = global_pos_msg->header;
    msg->pose.position.x = global_pos_msg->transform.translation.x;
    msg->pose.position.y = global_pos_msg->transform.translation.y;
    msg->pose.position.z = global_pos_msg->transform.translation.z;

    m_buf.lock();
    globalPosQueue.push(msg);
    m_buf.unlock();
}

//void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg, std::size_t* vio_cnt)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    //globalEstimator.inputOdom(t, vio_t, vio_q);

    // Test
    /*Eigen::Vector3d w_t = Eigen::Vector3d(4.688319, -1.786938, 0.783338);
    Eigen::Quaterniond w_q;
    w_q.x() = -0.153029;
    w_q.y() = -0.827283;
    w_q.z() = -0.082152;
    w_q.w() = 0.534108;

    vio_q = w_q * vio_q;
    vio_t = w_q.normalized().toRotationMatrix() * vio_t + w_t;
    globalEstimator.inputOdom(t, vio_t, vio_q);*/
    //end test


    m_buf.lock();
    while(!globalPosQueue.empty())
    {
        geometry_msgs::PoseStampedPtr gp_msg = globalPosQueue.front();
        double gp_t = gp_msg->header.stamp.toSec();
        //printf("vio t: %f, gp t: %f \n", t, gp_t);
        // 5ms sync tolerance
        if(gp_t >= t - 0.002 && gp_t <= t + 0.002)
        {
            printf("vio t: %f, gp t: %f \n", t, gp_t);
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            /*double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 1;*/
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
            //int numSats = GPS_msg->status.service;

            //Eigen::Matrix3d global_position_meas_cov;
            //global_position_meas_cov << 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6;

            // Debug
            //std::cout << "Im here 2\n";
            // end debug

            Eigen::Matrix<double, 3, 1> global_pos_measurement(gp_msg->pose.position.x,
                                                               gp_msg->pose.position.y,
                                                               gp_msg->pose.position.z);

            globalEstimator.inputGP(t, global_pos_measurement);
            globalPosQueue.pop();
            break;
        }
        else if(gp_t < t - 0.002)
            globalPosQueue.pop();
        else if(gp_t > t + 0.002)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    //publish_car_model(t, global_t, global_q);

    *vio_cnt += 1;
    if(*vio_cnt > 5)
    {
        // write result to file
        std::ofstream foutC("/home/rpg/stamped_traj_estimate.txt", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(20);
        foutC << pose_msg->header.stamp.toSec() << " ";
        foutC << global_t.x() << " "
              << global_t.y() << " "
              << global_t.z() << " "
              << global_q.x() << " "
              << global_q.y() << " "
              << global_q.z() << " "
              << global_q.w() << endl;
        foutC.close();
     }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    if(argc != 2)
    {
        printf("please intput: rosrun global_fusion global_fusion_node [config file] \n");
        return 1;
    }

    std::string config_file = argv[1];
    printf("global config_file: %s\n", argv[1]);

    // Global position measurements are provided as the position of P in world frame:
    // t_WP (or T_WP).
    std::string global_pos_topic;
    Eigen::Matrix4d T_BP;
    readParameters(config_file, global_pos_topic, T_BP);

    std::cout << "Global position topic: " << global_pos_topic << "\n";
    std::cout << "T_BP: \n" << T_BP << "\n";

    globalEstimator.setCalib(T_BP);

    global_path = &globalEstimator.global_path;

    /*if(global_pos_topic == "/leica/position")
    {
        ros::Subscriber sub_GP =
                n.subscribe("/leica/position", 100, leica_callback);
    }
    else if (global_pos_topic == "/vicon/firefly_sbx/firefly_sbx")
    {
        ros::Subscriber sub_GP =
                n.subscribe(global_pos_topic, 100, vicon_callback);
    }
    else
    {
        std::cout << "Unknown topic. \n";
    }*/

    ros::Subscriber sub_GP = n.subscribe("/ground_truth", 1, leica_callback);

    //ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 1, vio_callback);
    std::size_t vio_cnt = 0;
    ros::Subscriber sub_vio = n.subscribe<nav_msgs::Odometry>
            ("/vins_estimator/odometry", 1, boost::bind(&vio_callback, _1, &vio_cnt));

    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    //pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}
