/* Copyright (C) Imperial College, Smart Robotics Lab - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Dimos Tzoumanikas <dt214@ic.ac.uk>
 * 2015-2019
 */

#pragma once

#include <algorithm>
#include <atp/Datatypes.hpp>
#include <mpc/Helper.hpp>

namespace autopilot {

/**
 * @brief      Eigen::Vector3d Linear interpolation
 *
 * @param[in]  p_lowerBound  The lower bound
 * @param[in]  p_upperBound  The upper bound
 * @param[in]  p_percentage  The percentage
 *
 * @return     Interpolated 3D Vector
 */
inline Eigen::Vector3d interpolate3D(const Eigen::Vector3d& p_lowerBound,
                                     const Eigen::Vector3d& p_upperBound,
                                     const double p_percentage)
{
    return p_lowerBound + p_percentage * (p_upperBound - p_lowerBound);
}

/**
 * @brief      Eigen Quaternion Linear interpolation
 *
 * @param[in]  p_lowerBoundQuaternion  The lower bound quaternion
 * @param[in]  p_upperBoundQuaternion  The upper bound quaternion
 * @param[in]  p_percentage            The percentage
 *
 * @return     Interpolated Quaternion
 */
inline Eigen::Quaterniond interpolateOrientation(const Eigen::Quaterniond& p_lowerBoundQuaternion,
                                                 const Eigen::Quaterniond& p_upperBoundQuaternion,
                                                 const double p_percentage)
{
    return p_lowerBoundQuaternion.slerp(p_percentage, p_upperBoundQuaternion);
}

/**
 * @brief      Resamples An input trajectory
 *
 * @param[in]  p_inputTrajectory          The input trajectory
 * @param[in]  p_samplingTimeNanoseconds  The sampling time in nanoseconds
 * @param      p_resampledTrajectory      The resampled trajectory
 *
 * @return     True when successful
 */
inline bool resampleTrajectory(const Trajectory& p_inputTrajectory,
                               const uint64_t p_samplingTimeNanoseconds,
                               Trajectory& p_resampledTrajectory)
{
    // Make sure trajectory is empty
    p_resampledTrajectory.trajectory.clear();

    // Copy the timestamp
    p_resampledTrajectory.timestamp = p_inputTrajectory.timestamp;

    // Copy Id.
    p_resampledTrajectory.id = p_inputTrajectory.id;

    // Copy the flushQueue boolean
    p_resampledTrajectory.flushQueue = p_inputTrajectory.flushQueue;

    // Copy initial Waypoint
    p_resampledTrajectory.initialWaypoint = p_inputTrajectory.initialWaypoint;

    // Store timestamps in a different vector // TODO -> Double for loop is not
    // necessary. Finish the interpolation in a single loop
    std::vector<uint64_t> inputTrajectoryTimestamp;
    for (const SetpointStamped& SetpointStamped : p_inputTrajectory.trajectory) {
        inputTrajectoryTimestamp.push_back(
            SetpointStamped.timeNanoseconds
            - (*p_inputTrajectory.trajectory.begin()).timeNanoseconds);
    }

    if (!std::is_sorted(inputTrajectoryTimestamp.begin(), inputTrajectoryTimestamp.end())) {
        return false;
    }

    std::vector<uint64_t> resampledTrajectoryTimestamp;
    const uint64_t dt_Nanoseconds = p_samplingTimeNanoseconds;
    for (uint64_t timeNanoseconds = 0; timeNanoseconds <= inputTrajectoryTimestamp.back();
         timeNanoseconds += dt_Nanoseconds) {
        resampledTrajectoryTimestamp.push_back(timeNanoseconds);
    }

    // Can start the linear interpolation for the trajectory
    for (const uint64_t timeNanoseconds : resampledTrajectoryTimestamp) {
        // Determine previous and next iterator
        auto upperBoundTimestamp_it = std::upper_bound(
            inputTrajectoryTimestamp.begin(), inputTrajectoryTimestamp.end(), timeNanoseconds);
        // Check for equal or outside vector
        if (*std::prev(upperBoundTimestamp_it, 1) == timeNanoseconds
            || upperBoundTimestamp_it == inputTrajectoryTimestamp.end()) {
            // Decrease Iterator
            --upperBoundTimestamp_it;
        }

        auto lowerBoundTimestamp_it = std::lower_bound(
            inputTrajectoryTimestamp.begin(), inputTrajectoryTimestamp.end(), timeNanoseconds);
        if (*lowerBoundTimestamp_it != timeNanoseconds)
            --lowerBoundTimestamp_it;

        double percentage = 0.0;
        if (*upperBoundTimestamp_it != *lowerBoundTimestamp_it) {
            percentage = (timeNanoseconds - *(lowerBoundTimestamp_it))
                / static_cast<double>(*upperBoundTimestamp_it - *lowerBoundTimestamp_it);
        }

        // Find the setpoint in between
        const auto lowerBoundSetpoint = (lowerBoundTimestamp_it - inputTrajectoryTimestamp.begin())
            + p_inputTrajectory.trajectory.begin();
        const auto upperBoundSetpoint = (upperBoundTimestamp_it - inputTrajectoryTimestamp.begin())
            + p_inputTrajectory.trajectory.begin();

        // Perform linear interpolation for position
        const Eigen::Vector3d interpolated_position =
            interpolate3D(lowerBoundSetpoint->position, upperBoundSetpoint->position, percentage);

        // Perform linear interpolation for linear velocity
        const Eigen::Vector3d interpolated_linearVelocity = interpolate3D(
            lowerBoundSetpoint->linearVelocity, upperBoundSetpoint->linearVelocity, percentage);

        // Perform linear interpolation for linear acceleration
        const Eigen::Vector3d interpolated_linearAcceleration =
            interpolate3D(lowerBoundSetpoint->linearAcceleration,
                          upperBoundSetpoint->linearAcceleration,
                          percentage);

        // Perform Slerp for quaternion and get yaw
        Eigen::Quaterniond previous_quaternion, next_quaternion;
        controller::euler2quat(
            0.0, 0.0, controller::constrainAngle(lowerBoundSetpoint->yaw), previous_quaternion);
        controller::euler2quat(
            0.0, 0.0, controller::constrainAngle(upperBoundSetpoint->yaw), next_quaternion);
        const Eigen::Quaterniond interpolated_quaternion =
            interpolateOrientation(previous_quaternion, next_quaternion, percentage);
        double roll_interpolated, pitch_interpolated, yaw_interpolated;
        controller::quat2euler(
            interpolated_quaternion, roll_interpolated, pitch_interpolated, yaw_interpolated);

        // Perform linear interpolation for yaw
        // const double yaw_interpolated = lowerBoundSetpoint->yaw + (percentage *
        // (upperBoundSetpoint->yaw - lowerBoundSetpoint->yaw));

        // Store everything in a new vector
        p_resampledTrajectory.trajectory.push_back(SetpointStamped(interpolated_position,
                                                                   interpolated_linearVelocity,
                                                                   interpolated_linearAcceleration,
                                                                   yaw_interpolated,
                                                                   timeNanoseconds));
    }
    return true;
}

/**
 * @brief      Generates a straight Line trajectory
 *
 * @param[in]  p_initialPosition          The initial position
 * @param[in]  p_finalPosition            The final position
 * @param[in]  p_linearVelocity           The linear velocity
 * @param[in]  p_yaw                      The const yaw
 * @param[in]  p_positionTolerance        The position tolerance for the initial
 *                                        waypoint
 * @param[in]  p_orientationTolerance     The orientation tolerance
 * @param[in]  p_samplingTimeNanoseconds  The sampling time in nanoseconds
 * @param      p_trajectory               The resampled trajectory
 *
 * @return     True when successful
 */
inline bool generateStraightTrajectory(const Eigen::Vector3d& p_initialPosition,
                                       const Eigen::Vector3d& p_finalPosition,
                                       const double p_linearVelocity,
                                       const double p_yaw,
                                       const double p_positionTolerance,
                                       const double p_orientationTolerance,
                                       const uint64_t p_samplingTimeNanoseconds,
                                       Trajectory& p_trajectory)
{
    // Straight Line trajectory with two points
    Trajectory straightLineTrajectory;
    straightLineTrajectory.initialWaypoint.position = p_initialPosition;
    straightLineTrajectory.initialWaypoint.yaw = p_yaw;
    straightLineTrajectory.initialWaypoint.positionTolerance = p_positionTolerance;
    straightLineTrajectory.initialWaypoint.orientationTolerance = p_orientationTolerance;

    uint64_t finalTimeNanoseconds = 0;

    // Check p_linearVelocity
    Eigen::Vector3d resampledReferenceVelocity =
        (p_finalPosition - p_initialPosition).normalized() * fabs(p_linearVelocity);

    if (fabs(p_linearVelocity) <= 1 * 1e-3) {
        finalTimeNanoseconds = 1e+9 * (p_finalPosition - p_initialPosition).norm() / 0.50;
    }
    else {
        finalTimeNanoseconds =
            1e+9 * (p_finalPosition - p_initialPosition).norm() / fabs(p_linearVelocity);
    }

    // Use Zero velocities for initial Tests
    //resampledReferenceVelocity =
    //   Eigen::Vector3d::Zero(); // ToDo -> remove this and debug

    // First point of the trajectory
    const SetpointStamped first_point(
        p_initialPosition, resampledReferenceVelocity, Eigen::Vector3d::Zero(), p_yaw, 0);

    // Final point of the trajectory
    const SetpointStamped final_point(p_finalPosition,
                                      resampledReferenceVelocity,
                                      Eigen::Vector3d::Zero(),
                                      p_yaw,
                                      finalTimeNanoseconds);
    straightLineTrajectory.trajectory.push_back(first_point);
    straightLineTrajectory.trajectory.push_back(final_point);

    // Resample the trajectory
    return resampleTrajectory(straightLineTrajectory, p_samplingTimeNanoseconds, p_trajectory);
}

} // namespace autopilot
