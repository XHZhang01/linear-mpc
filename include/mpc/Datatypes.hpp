/* Copyright (C) Imperial College, Smart Robotics Lab - All Rights Reserved
 * Unauthorised copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Dimos Tzoumanikas <dt214@ic.ac.uk>
 * 2015-2019
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <vector>

namespace controller {

/**
 * @brief      Enum for the different type of references
 */
enum class ReferenceType { kSetpoint = 0, kTrajectory };

/**
 * @brief      Class for the different reference frames (world or body).
 */
enum class ReferenceFrame { kWorld = 0, kBody };

/**
 * @brief      Timestamp
 */
struct Timestamp {
    uint32_t seconds;
    uint32_t nanoSeconds;
    Timestamp(const uint32_t p_seconds = 0, const uint32_t p_nanoSeconds = 0) :
            seconds(p_seconds), nanoSeconds(p_nanoSeconds){};
};

/**
 * @brief      Contains the data provided from the State estimator
 */
struct MavState {
    Timestamp timestamp;                 ///< Timestamp
    Eigen::Vector3d position;            ///< r_WB. Position in World frame
    Eigen::Vector3d linearVelocity;      ///< v_F_WB. Position wrt to World frame expressed in
                                         ///< frame F. With {F} == linearVelocityFrame;
    Eigen::Quaterniond orientation;      ///< q_WB. Orientation quaternion
    Eigen::Vector3d angularVelocity;     ///< Angular velocity expressed in the
                                         ///< angularVelocityFrame
    ReferenceFrame linearVelocityFrame;  ///< Frame used for the linear velocity
    ReferenceFrame angularVelocityFrame; ///< Frame used for the angular velocity

    /**
   * @brief      Constructs a new MAV state instance.
   *
   * @param[in]  p_timestamp             The timestamp
   * @param[in]  p_position              Position. r_WB
   * @param[in]  p_linearVelocity        Lin. velocity v_F_WB
   * @param[in]  p_orientation           The orientation quaternion q_WB
   * @param[in]  p_angularVelocity       The angular velocity
   * @param[in]  p_linearVelocityFrame   The linear velocity frame
   * @param[in]  p_angularVelocityFrame  The angular velocity frame
   */
    MavState(const Timestamp p_timestamp = Timestamp(0, 0),
             const Eigen::Vector3d& p_position = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d& p_linearVelocity = Eigen::Vector3d::Zero(),
             const Eigen::Quaterniond& p_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
             const Eigen::Vector3d& p_angularVelocity = Eigen::Vector3d::Zero(),
             const ReferenceFrame p_linearVelocityFrame = ReferenceFrame::kWorld,
             const ReferenceFrame p_angularVelocityFrame = ReferenceFrame::kWorld) :
            timestamp(p_timestamp),
            position(p_position),
            linearVelocity(p_linearVelocity),
            orientation(p_orientation),
            angularVelocity(p_angularVelocity),
            linearVelocityFrame(p_linearVelocityFrame),
            angularVelocityFrame(p_angularVelocityFrame){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief      Contains the Reference state passed to the controller
 */
struct ReferenseSetpoint {
    Eigen::Vector3d position;           ///< r_WBr. Reference position in World frame.
    Eigen::Vector3d linearVelocity;     ///< v_WBr. Reference linear velocity in World
    double yaw;                         ///< Reference yaw.
    Eigen::Vector3d linearAcceleration; ///< a_WBr. Reference linear acc in World frame

    ReferenseSetpoint(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
                      const Eigen::Vector3d& linearVelocity = Eigen::Vector3d::Zero(),
                      const double yaw = 0.0,
                      const Eigen::Vector3d& linearAcceleration = Eigen::Vector3d::Zero()) :
            position(position),
            linearVelocity(linearVelocity),
            yaw(yaw),
            linearAcceleration(linearAcceleration){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// MPC Queue. -> Deque with Reference States
typedef std::deque<ReferenseSetpoint, Eigen::aligned_allocator<ReferenseSetpoint>> MPCQueue;

// Reference Trajectory -> Vector with Reference States
typedef std::vector<ReferenseSetpoint, Eigen::aligned_allocator<ReferenseSetpoint>>
    ReferenceTrajectory;

/**
 * @brief      Struct for the control command
 */
struct ControlCommands {
    Timestamp timestamp;            ///< Timestamp
    Eigen::Quaterniond orientation; ///< Reference orientation in the PX4 frame
    double thrust;                  ///< Thrust (excluding the Feed forward term) in g

    /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  timestamp    The timestamp
   * @param[in]  orientation  The orientation quaternion
   * @param[in]  thrust       The thrust
   */
    ControlCommands(const Timestamp timestamp = Timestamp(0, 0),
                    const Eigen::Quaterniond orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                    const double thrust = 0.0) :
            timestamp(timestamp), orientation(orientation), thrust(thrust){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief      Discrete Low Pass filter
 */
class LowPassFilter {
    public:
    LowPassFilter(const double p_cutoffFrequency = 0.0) : m_cutoffFrequency(p_cutoffFrequency)
    {
    }
    ~LowPassFilter()
    {
    }

    double getFilteredValue(const double p_input, const double p_timestampSeconds)
    {
        // Update alpha
        const double alpha = computeAlpha(p_timestampSeconds);

        // Compute Low pass value
        m_output = alpha * p_input + (1.0 - alpha) * m_output;

        // Remember previous input and time
        m_timestampSeconds = p_timestampSeconds;
        return m_output;
    }

    private:
    double computeAlpha(const double p_timestampSeconds)
    {
        if (m_cutoffFrequency <= 0.0 || m_timestampSeconds == 0.0) {
            return 1.0;
        }
        else {
            const double dt = (p_timestampSeconds - m_timestampSeconds);
            if (dt < 0) {
                // Received input from the past
                // std::cout << "Low pass received input from the past." << std::endl;
                return 0.0;
            }
            else {
                return (2 * M_PI * dt * m_cutoffFrequency)
                    / (2 * M_PI * dt * m_cutoffFrequency + 1);
            }
        }
    }

    const double m_cutoffFrequency = 0.0; ///< Low pass frequency in Hz
    double m_timestampSeconds = 0.0;      ///< Timestamp in seconds of last update
    double m_output = 0.0;                ///< Previous filter update
};

} // namespace controller
