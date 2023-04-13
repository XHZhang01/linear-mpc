#pragma once

#include <Eigen/Core>
#include <chrono>
#include <cstring>
#include <iostream>
#include <math.h>
#include <mpc/Constants.hpp>
#include <mpc/Datatypes.hpp>
//#include <unsupported/Eigen/KroneckerProduct>
//#include <unsupported/Eigen/MatrixFunctions>

namespace controller {

/**
 * @brief      Constraints the input angle to be in the [-pi,pi) range
 *
 * @param[in]  angle  The input angle in radians
 *
 * @return     The constrained angle
 */
inline double constrainAngle(const double angle)
{
    double clampedAngle = angle;
    while (clampedAngle <= -M_PI)
        clampedAngle += 2 * M_PI;
    while (clampedAngle > M_PI)
        clampedAngle -= 2 * M_PI;
    return clampedAngle;
}

/**
 * @brief      Converts degrees to radians
 *
 * @param[in]  angle  Angle in degrees
 *
 * @return     Angle in radians
 */
inline double deg2rad(const double angle)
{
    return angle * 0.01744444444;
}

/**
 * @brief      Converts radians to degrees
 *
 * @param[in]  angle  Angle in radians
 *
 * @return     Angle in degrees
 */
inline double rad2deg(const double angle)
{
    return angle * 57.32484076;
}

/**
 * @brief      Converts an input quaternion to Euler angles (ZYX)
 *
 * @param[in]  q      The input Quaternion
 * @param      roll   Roll angle
 * @param      pitch  Pitch angle
 * @param      yaw    Yaw angle
 */
inline void quat2euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    const double ysqr = q.y() * q.y();

    const double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
    const double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
    roll = std::atan2(t0, t1);

    const double t2 = std::min(std::max(+2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0), 1.0);

    // Check close greater to
    if (std::fabs(t2) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, t2);
    }

    pitch = std::asin(t2);

    const double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
    const double t4 = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = std::atan2(t3, t4);
}

/**
 * @brief      Returns the yaw angle
 *
 * @param[in]  q     The quaternion
 *
 * @return     The yaw angle
 */
inline double quat2yaw(const Eigen::Quaterniond& q)
{
    const double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
    const double t4 = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(t3, t4);
}

/**
 * @brief      Converts euler angles (ZYX convention) to a quaternion
 *
 * @param[in]  roll   Phi
 * @param[in]  pitch  Theta
 * @param[in]  yaw    Psi
 * @param      q      Quaternion
 */
inline void
euler2quat(const double roll, const double pitch, const double yaw, Eigen::Quaterniond& q)
{
    const double t0 = std::cos(yaw * 0.5);
    const double t1 = std::sin(yaw * 0.5);
    const double t2 = std::cos(roll * 0.5);
    const double t3 = std::sin(roll * 0.5);
    const double t4 = std::cos(pitch * 0.5);
    const double t5 = std::sin(pitch * 0.5);

    q.w() = t0 * t2 * t4 + t1 * t3 * t5;
    q.x() = t0 * t3 * t4 - t1 * t2 * t5;
    q.y() = t0 * t2 * t5 + t1 * t3 * t4;
    q.z() = t1 * t2 * t4 - t0 * t3 * t5;
}

/**
 * @brief      Converts a 3D vector expressed in the World frame to a 3D vector
 * in Body frame
 *
 * @param[in]  q_WB   The orientation quaternion
 * @param[in]  vec_W  The 3D vector expressed in the World frame
 * @param      vec_B  The 3D vector expressed in the Body frame
 */
inline void
world2body(const Eigen::Quaterniond& q_WB, const Eigen::Vector3d& vec_W, Eigen::Vector3d& vec_B)
{
    vec_B = q_WB.toRotationMatrix().inverse() * vec_W;
}

/**
 * @brief      Converts a 3D vector expressed in the Body frame to a 3D vector
 * in World frame
 *
 * @param[in]  q_WB   The orientation quaternion
 * @param[in]  vec_B  The 3D vector expressed in the Body frame
 * @param      vec_W  The 3D vector expressed in the World frame
 */
inline void
body2world(const Eigen::Quaterniond& q_WB, const Eigen::Vector3d& vec_B, Eigen::Vector3d& vec_W)
{
    vec_W = q_WB.toRotationMatrix() * vec_B;
}

/**
 * @brief      Converts a 3D vector expressed in the World frame to a vector in
 * the navigation frame used for control
 *
 * @param[in]  q_WB   The orientation quaternion
 * @param[in]  vec_W  The 3D vector expressed in the World frame
 *
 * @return     The 3D vector expressed in the Navigation frame
 */
inline Eigen::Vector3d world2nav(const Eigen::Quaterniond& q_WB, const Eigen::Vector3d& vec_W)
{
    double roll, pitch, yaw;
    quat2euler(q_WB, roll, pitch, yaw);
    Eigen::Matrix3d C_WN;

    // clang-format off
  C_WN << std::cos(yaw), -std::sin(yaw), 0.0,
          std::sin(yaw),  std::cos(yaw), 0.0,
                    0.0,            0.0, 1.0;

    // clang-format on

    return C_WN.transpose() * vec_W;
}

/**
 * @brief      Converts a 3D vector expressed in the Navigation frame to a
 *             vector in the World frame
 *
 * @param[in]  q_WB   The orientation quaternion
 * @param[in]  vec_N  The 3D vector in the navigation frame
 *
 * @return     The 3D vector in the World frame
 */
inline Eigen::Vector3d nav2world(const Eigen::Quaterniond& q_WB, const Eigen::Vector3d& vec_N)
{
    double roll, pitch, yaw;
    quat2euler(q_WB, roll, pitch, yaw);
    Eigen::Matrix3d C_WN;

    // clang-format off
  C_WN << std::cos(yaw), -std::sin(yaw), 0.0,
          std::sin(yaw),  std::cos(yaw), 0.0,
                    0.0,            0.0, 1.0;

    // clang-format on

    return C_WN * vec_N;
}

} // namespace controller
