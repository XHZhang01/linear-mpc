#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <algorithm>
#include <boost/variant.hpp>
#include <mav_interface_msgs/conversions.h>
#include <mpc/Helper.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace autopilot {

/**
 * @brief      Enum for the high level commands sent to the autopilot
 */
enum class HighLevelCmd {
    kIdle = 0,
    kTakeOff,
    kLand,
    kReturn2home,
    kLandhome,
    kPause,
    kResume,
    kArm,
    kDisarm
};

/**
 * @brief      Waypoint struct. Contains reference position, yaw and position
 * tolerance
 */
struct Waypoint {
    Eigen::Vector3d position;    // Reference position
    double yaw;                  // Reference yaw
    double positionTolerance;    // Waypoint position tolerance
    double orientationTolerance; // Waypoint orientation tolerance

    /**
   * @brief      Constructs a new instance.
   */
    Waypoint() :
            position(0.0, 0.0, 0.0), yaw(0.0), positionTolerance(0.3), orientationTolerance(0.3)
    {
    }

    /**
   * @brief      Constructs a new instance from a Waypoint ROS mmessage
   *
   * @param[in]  waypointMsg  The waypoint ROS message
   */
    Waypoint(const mav_interface_msgs::Waypoint& waypointMsg) :
            position(Eigen::Vector3d(waypointMsg.position.x,
                                     waypointMsg.position.y,
                                     waypointMsg.position.z)),
            yaw(controller::quat2yaw(Eigen::Quaterniond(waypointMsg.orientation.w,
                                                        waypointMsg.orientation.x,
                                                        waypointMsg.orientation.y,
                                                        waypointMsg.orientation.z)
                                         .normalized())),
            positionTolerance(waypointMsg.positionTolerance),
            orientationTolerance(waypointMsg.orientationTolerance){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief      Path is a vector of waypoints
 */
struct Path {
    using Waypoints = std::vector<Waypoint, Eigen::aligned_allocator<Waypoint>>;

    controller::Timestamp timestamp; // Timestamp
    std::string id;                  // Task Id
    bool flushQueue;                 // Flush The Queue or Not ?
    Waypoints waypoints;             // A path is a sequence of waypoints

    Path(const mav_interface_msgs::Path& pathMsg) :
            timestamp(pathMsg.header.stamp.sec, pathMsg.header.stamp.nsec),
            id(pathMsg.taskID),
            flushQueue(pathMsg.flushReferenceQueue)
    {
        waypoints.clear();
        // Iterate all the Waypoints and append
        for (const auto& waypointMsg : pathMsg.waypoints) {
            waypoints.push_back(Waypoint(waypointMsg));
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief      Trajectory setpoint with position, velocity acceleration data
 */
struct SetpointStamped {
    uint64_t timeNanoseconds;           // Relative timestasmp
    Eigen::Vector3d position;           // Reference position
    Eigen::Vector3d linearVelocity;     // Reference linearVelocity
    Eigen::Vector3d linearAcceleration; // Reference Linear acceleration
    double yaw;                         // Reference yaw

    SetpointStamped(const Eigen::Vector3d& p_position = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& p_linearVelocity = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d& p_linearAcceleration = Eigen::Vector3d::Zero(),
                    const double p_yaw = 0.0,
                    const uint64_t p_timeNanoseconds = 0) :
            timeNanoseconds(p_timeNanoseconds),
            position(p_position),
            linearVelocity(p_linearVelocity),
            linearAcceleration(p_linearAcceleration),
            yaw(p_yaw)
    {
    }

    SetpointStamped(const mav_interface_msgs::FullStateStamped& fullStateStampedMsg) :
            timeNanoseconds(fullStateStampedMsg.timestampNanoSeconds),
            position(Eigen::Vector3d(fullStateStampedMsg.position.x,
                                     fullStateStampedMsg.position.y,
                                     fullStateStampedMsg.position.z)),
            linearVelocity(Eigen::Vector3d(fullStateStampedMsg.linearVelocity.x,
                                           fullStateStampedMsg.linearVelocity.y,
                                           fullStateStampedMsg.linearVelocity.z)),
            linearAcceleration(Eigen::Vector3d(fullStateStampedMsg.linearAcceleration.x,
                                               fullStateStampedMsg.linearAcceleration.y,
                                               fullStateStampedMsg.linearAcceleration.z)),
            yaw(controller::quat2yaw(Eigen::Quaterniond(fullStateStampedMsg.orientation.w,
                                                        fullStateStampedMsg.orientation.x,
                                                        fullStateStampedMsg.orientation.y,
                                                        fullStateStampedMsg.orientation.z)
                                         .normalized())){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief      Trajectory is a sequence of trajectory set-points + an initial
 * waypoint
 */
struct Trajectory {
    using SetpointStampedVector =
        std::vector<SetpointStamped, Eigen::aligned_allocator<SetpointStamped>>;

    controller::Timestamp timestamp;  // Timestamp
    std::string id;                   // Task Id
    bool flushQueue;                  // Empty the Reference queue ?
    Waypoint initialWaypoint;         // Initial waypoint before starting the actual trajectory
    SetpointStampedVector trajectory; // A full trajectory is a vector of trajectory setpoints

    /**
   * @brief      Constructs a new instance.
   */
    Trajectory() : timestamp(0, 0), id("null"), flushQueue(false), initialWaypoint(){};

    /**
   * @brief      Constructs a new instance from a Trajectory ROS msg.
   *
   * @param[in]  fullstateTrajectoryMsg  The fullstate ROS trajectory message
   */
    Trajectory(const mav_interface_msgs::FullStateTrajectory& fullstateTrajectoryMsg) :
            timestamp(fullstateTrajectoryMsg.header.stamp.sec,
                      fullstateTrajectoryMsg.header.stamp.nsec),
            id(fullstateTrajectoryMsg.taskID),
            flushQueue(fullstateTrajectoryMsg.flushReferenceQueue),
            initialWaypoint(Waypoint(fullstateTrajectoryMsg.initialWaypoint))
    {
        // Clear the vector
        trajectory.clear();

        // Iterate the reference Msg and append to the vector.
        for (const auto& stateStampedMsg : fullstateTrajectoryMsg.trajectory) {
            trajectory.push_back(SetpointStamped(stateStampedMsg));
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief      Reference Command for the autopilot queue is either a waypoint or
 * a trajectory
 */
typedef boost::variant<Waypoint, Trajectory> ReferenceCommand;

/**
 * @brief      Autopilot Queue
 */
typedef std::deque<ReferenceCommand, Eigen::aligned_allocator<ReferenceCommand>> AutopilotQueue;

/**
 * @brief      Struct for the MAV state
 */
struct MavState : controller::MavState {
    /**
   * @brief      Constructor from odometry message
   *
   * @param      p_odometry  The p odometry
   */
    MavState(const nav_msgs::OdometryConstPtr& p_odometry,
             const controller::ReferenceFrame p_linearVelocityFrame,
             const controller::ReferenceFrame p_angularVelocityFrame) :
            controller::MavState(
                controller::Timestamp(p_odometry->header.stamp.sec, p_odometry->header.stamp.nsec),
                Eigen::Vector3d(p_odometry->pose.pose.position.x,
                                p_odometry->pose.pose.position.y,
                                p_odometry->pose.pose.position.z),
                Eigen::Vector3d(p_odometry->twist.twist.linear.x,
                                p_odometry->twist.twist.linear.y,
                                p_odometry->twist.twist.linear.z),
                Eigen::Quaterniond(p_odometry->pose.pose.orientation.w,
                                   p_odometry->pose.pose.orientation.x,
                                   p_odometry->pose.pose.orientation.y,
                                   p_odometry->pose.pose.orientation.z)
                    .normalized(),
                Eigen::Vector3d(p_odometry->twist.twist.angular.x,
                                p_odometry->twist.twist.angular.y,
                                p_odometry->twist.twist.angular.z),
                p_linearVelocityFrame,
                p_angularVelocityFrame){};
};

/**
 * @brief      Struct for the MPC reference
 */
struct ReferenceState : controller::ReferenseSetpoint {
    /**
   * @brief      Constructor from waypoint
   *
   * @param[in]  p_waypoint  The reference waypoint
   */
    ReferenceState(const Waypoint& p_waypoint) :
            ReferenseSetpoint(p_waypoint.position, Eigen::Vector3d::Zero(), p_waypoint.yaw){};

    /**
   * @brief      Constructor from Trajectory point
   *
   * @param[in]  p_trajectorySngl  The trajectory point
   */
    ReferenceState(const SetpointStamped& p_trajectorySngl) :
            ReferenseSetpoint(p_trajectorySngl.position,
                              p_trajectorySngl.linearVelocity,
                              p_trajectorySngl.yaw){};
};

} // namespace autopilot
