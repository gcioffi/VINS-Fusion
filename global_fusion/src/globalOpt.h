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

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
    void setCalib(Eigen::Matrix4d& T_BP);
    void set_T_WVIO(Eigen::Vector3d& t_WVIO, Eigen::Quaterniond& q_WVIO);
    void inputGP(double t, Eigen::Vector3d position);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
    void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ, double& last_ts);
    int getOptimizationId(){
        return optimization_cnt_;
    }
    void get_T_WVIO(Eigen::Vector3d& t_WVIO, Eigen::Quaterniond& q_WVIO)
    {
        t_WVIO = W_T_WVIO.block<3,1>(0,3);

        Eigen::Matrix3d R_WVIO = W_T_WVIO.block<3,3>(0,0);
        Eigen::Quaterniond q(R_WVIO);
        q_WVIO = q;
    }
	nav_msgs::Path global_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void updateGlobalPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
    map<double, vector<double>> globalPositionMap;
	bool initGPS;
    bool newGP;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
    Eigen::Matrix4d W_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
    double lastTimestamp;
	std::thread threadOpt;

    Eigen::Matrix4d T_BP_;
    Eigen::Matrix4d T_PB_;
    double global_position_meas_square_root_cov_ = 0.001;
    int optimization_cnt_ = 0;

};
