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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
double last_vio_t = -1;
double last_gp_t = -1;
std::queue<geometry_msgs::PoseStamped> globalPosQueue;
std::queue<geometry_msgs::PoseStamped> estimatedPoseQueue;
std::vector<bool> isKeyframe;
double gp_rate = 1.5; // Rate with which gp measurements are inserted in the optimization.
std::size_t gp_cnt = 0;
bool global_optimization_started = false;

void readParameters(std::string config_file, std::string& sequence, int &first_gp_meas)
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

    fsSettings["sequence"] >> sequence;
    first_gp_meas = fsSettings["first_gp_meas"];
}

void loadGroundTruth(std::string& filename, int first_meas_id)
{
    std::ifstream fs(filename.c_str());
    if(!fs.is_open())
    {
      std::cout << "Could not open ground truth file: " << filename << "\n";
    }

    // add all gt positions
    size_t n = 0;

    while(fs.good() && !fs.eof())
    {
      if(fs.peek() == '#') // skip comments
        fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      int id;
      double stamp, x, y, z, qx, qy, qz, qw;
      fs >> id >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw;

      if(id >= first_meas_id)
      {
        const Eigen::Vector3d position(x, y, z);
        const Eigen::Quaterniond orientation(qw, qx, qy, qz);

        geometry_msgs::PoseStamped msg;

        std_msgs::Header msg_header;
        ros::Time t;
        //msg_header.seq = n;
        msg_header.stamp = t.fromSec(stamp);
        //msg_header.frame_id = "";

        msg.header = msg_header;
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.x = qx;
        msg.pose.orientation.y = qy;
        msg.pose.orientation.z = qz;
        msg.pose.orientation.w = qw;

        globalPosQueue.push(msg);

        // Debug
        /*std::cout << "id, stamp, x, y, z, qx, qy, qz, qw\n";
        std::cout << id << ", "
                  << stamp << ", "
                  << x << ", "
                  << y << ", "
                  << z << ", "
                  << qx << ", "
                  << qy << ", "
                  << qz << ", "
                  << qw << "\n";*/
        // end Debug
        ++n;
      }
    }

    std::cout << "Loaded " << globalPosQueue.size() << " gt measurements.\n";
}

void loadPoseEstimates(std::string& filename)
{
    std::ifstream fs(filename.c_str());
    if(!fs.is_open())
    {
      std::cout << "Could not open pose estimates file: " << filename << "\n";
    }

    // add all gt positions
    size_t n = 0;

    while(fs.good() && !fs.eof())
    {
      if(fs.peek() == '#') // skip comments
        fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      double stamp, x, y, z, qx, qy, qz, qw;
      fs >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw;

      const Eigen::Vector3d position(x, y, z);
      const Eigen::Quaterniond orientation(qw, qx, qy, qz);

      geometry_msgs::PoseStamped msg;

      std_msgs::Header msg_header;
      ros::Time t;
      msg_header.stamp = t.fromSec(stamp);

      msg.header = msg_header;
      msg.pose.position.x = x;
      msg.pose.position.y = y;
      msg.pose.position.z = z;
      msg.pose.orientation.x = qx;
      msg.pose.orientation.y = qy;
      msg.pose.orientation.z = qz;
      msg.pose.orientation.w = qw;

      estimatedPoseQueue.push(msg);

      // Debug
      /*std::cout << "id, stamp, x, y, z, qx, qy, qz, qw\n";
      std::cout << id << ", "
                << stamp << ", "
                << x << ", "
                << y << ", "
                << z << ", "
                << qx << ", "
                << qy << ", "
                << qz << ", "
                << qw << "\n";*/
      // end Debug
      ++n;
    }

    std::cout << "Loaded " << estimatedPoseQueue.size() << " estimates.\n";
}

void loadFrameStatus(std::string& filename)
{
    std::ifstream fs(filename.c_str());
    if(!fs.is_open())
    {
      std::cout << "Could not open status file: " << filename << "\n";
    }

    // add all gt positions
    size_t n = 0;

    while(fs.good() && !fs.eof())
    {
      if(fs.peek() == '#') // skip comments
        fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      int id;
      double stamp;
      std::string vio_status;
      std::string tracking_quality;
      std::string frame_status;
      fs >> id >> stamp >> vio_status >> tracking_quality >> frame_status;

      if(frame_status == "KF")
      {
          isKeyframe.push_back(true);
      }
      else
      {
          isKeyframe.push_back(false);
      }

      n++;
    }
}

void callback(const geometry_msgs::PoseStamped pose_msg, std::string& global_pe_fn)
{
    //printf("vio_callback! \n");
    double t = pose_msg.header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg.pose.position.x,
                          pose_msg.pose.position.y,
                          pose_msg.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg.pose.orientation.w;
    vio_q.x() = pose_msg.pose.orientation.x;
    vio_q.y() = pose_msg.pose.orientation.y;
    vio_q.z() = pose_msg.pose.orientation.z;

    globalEstimator.inputOdom(t, vio_t, vio_q);

    while(!globalPosQueue.empty())
    {
        geometry_msgs::PoseStamped gp_msg = globalPosQueue.front();
        double gp_t = gp_msg.header.stamp.toSec();
        //printf("vio t: %f, gp t: %f \n", t, gp_t);

        // 1ms sync tolerance
        if(gp_t >= t - 0.001 && gp_t <= t + 0.001)
        {
            /*if(gp_t >= (last_gp_t + 1.0/gp_rate))
            {
                //printf("vio t: %f, gp t: %f \n", t, gp_t);
                Eigen::Matrix<double, 3, 1> global_pos_measurement(gp_msg.pose.position.x,
                                                                   gp_msg.pose.position.y,
                                                                   gp_msg.pose.position.z);

                globalEstimator.inputGP(t, global_pos_measurement);
                globalPosQueue.pop();

                last_gp_t = gp_t;
                gp_cnt++;
            }*/
            Eigen::Matrix<double, 3, 1> global_pos_measurement(gp_msg.pose.position.x,
                                                               gp_msg.pose.position.y,
                                                               gp_msg.pose.position.z);

            globalEstimator.inputGP(t, global_pos_measurement);
            globalPosQueue.pop();

            gp_cnt++;

            break;
        }
        else if(gp_t < t - 0.001)
            globalPosQueue.pop();
        else if(gp_t > t + 0.001)
            break;
    }

    if (!global_optimization_started)
    {
        if(globalEstimator.getOptimizationId() > 0)
        {
            global_optimization_started = true;
        }
    }

    if (global_optimization_started)
    {
        Eigen::Vector3d global_t;
        Eigen:: Quaterniond global_q;
        globalEstimator.getGlobalOdom(global_t, global_q);

        /*nav_msgs::Odometry odometry;
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
        pub_global_path.publish(*global_path);*/

        // write result to file
        std::ofstream foutC(global_pe_fn, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(20);
        foutC << pose_msg.header.stamp.toSec() << " ";
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

    // Global position measurements are provided as the position of B in world frame:
    std::string sequence_id;
    int first_gp_meas = 0;
    readParameters(config_file, sequence_id, first_gp_meas);

    std::cout << "first id " << first_gp_meas << "\n";

    //global_path = &globalEstimator.global_path;

    // Load gt.
    std::string gt_fn(
                "/home/rpg/vins-fusion_ws/src/VINS-Fusion/"
                "cvpr2020_comparison/" + sequence_id + "/groundtruth.txt");
    loadGroundTruth(gt_fn, first_gp_meas);

    std::string pe_fn(
                "/home/rpg/vins-fusion_ws/src/VINS-Fusion/"
                "cvpr2020_comparison/" + sequence_id + "/svo_traj_estimate.txt");
    loadPoseEstimates(pe_fn);

    /*std::string status_fn(
                "/home/rpg/vins-fusion_ws/src/VINS-Fusion/"
                "cvpr2020_comparison/" + sequence_id + "/svo_traj_estimate.txt");
    loadFrameStatus(pe_fn);*/

    std::string global_pe_fn("/home/rpg/vins-fusion_ws/src/VINS-Fusion/"
                             "cvpr2020_comparison/" + sequence_id +
                             "/stamped_traj_estimate.txt");
    // Remove existing file
    if(std::remove(global_pe_fn.c_str()) != 0)
    {
        std::cout << "No previous existing file\n";
    }
    else
    {
        std::cout << "Previous existing file removed\n";
    }

    // Compute initial T_W_Wvio_init
    //Eigen::Matrix4d T_W_Wvio_init;
    //compute_TW_Wvio_init(T_W_Wvio_init);

    // Simulated callback
    while(estimatedPoseQueue.size()>0)
    {
        callback(estimatedPoseQueue.front(), global_pe_fn);
        estimatedPoseQueue.pop();

        // Wait optimization is complete.
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }

    std::cout << "Num. gp fused: " << gp_cnt << "\n";

    //ros::Subscriber sub_GP = n.subscribe("/svo/backend_pose_imu", 1, leica_callback);

    //ros::Subscriber sub_vio = n.subscribe("/svo/pose_imu", 1, vio_callback);
    //std::size_t vio_cnt = 0;
    //ros::Subscriber sub_vio = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>
            //("/svo/backend_pose_imu", 1, boost::bind(&vio_callback, _1, vio_cnt));

    //pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    //pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    //pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);

    //ros::spin();
    ros::shutdown();
    return 0;
}
