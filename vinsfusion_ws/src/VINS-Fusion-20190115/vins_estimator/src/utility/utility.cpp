/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

/*
 * 从加速度计测量重力分量得到姿态矩阵(yaw = 0)
 */
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();//得到旋转矩阵
    double yaw = Utility::R2ypr(R0).x();//计算旋转矩阵对应欧拉角yaw
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;//抵消掉yaw的旋转,得到pr构建的旋转矩阵
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
