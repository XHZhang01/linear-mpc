/* Copyright (C) Imperial College, Smart Robotics Lab - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Dimos Tzoumanikas <dt214@ic.ac.uk>
 * 2015-2019
 */

#pragma once
#include <boost/sml.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <mav_interface_msgs/AutopilotStatusService.h>
#include <mav_interface_msgs/conversions.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
//#include <model/StateSpaceModel.hpp>
#include <atp/AutopilotStateMachine.hpp>
#include <atp/FullPoseStateMachine.hpp>
#include <atp/Helper.hpp>
#include <model/ThrustModel.hpp>
#include <mpc/Helper.hpp>
#include <mpc/MPC.hpp>
#include <mpc_ros/CommandsConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

namespace autopilot {

class Autopilot {
    public:

    struct Parameters {
        double take_off_height = 1.5;
        double take_off_velocity = 0.2;
        double landing_height = -0.2;
        double landing_velocity = 0.2;
        Eigen::Vector3d home_position = Eigen::Vector3d(0.0, 0.0, -0.2);
        double return_home_height = 0.2;
        double return_home_velocity = 0.2;

        Parameters() = delete;

        Parameters(const std::string filename);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // Delete default constructor
    Autopilot() = delete;

    ~Autopilot(){};

    Autopilot(ros::NodeHandle& p_nh,
              const Parameters& parameters,
              const controller::FirstOrderModel<controller::kHorizonLength>& p_state_space_model,
              const std::shared_ptr<AutopilotQueue>& p_atpQueuePtr,
              const std::shared_ptr<controller::MPCQueue>& p_mpcQueuePtr,
              const std::shared_ptr<controller::Px4ThrustModel>& thrustModelPtr);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
     * @brief      Arms the motors
     */
    void armMotors();

    /**
     * @brief      Updates the battery voltage
     *
     * @param[in]  p_batteryStateMsg  The battery message
     */
    void batteryStateCallback(const sensor_msgs::BatteryStatePtr& p_batteryStateMsg);

    /**
     * @brief      Dynamic reconfigure callback
     *
     * @param      p_config  The configuration
     * @param[in]  p_level   The level
     */
    void dynamicReconfigureCallback(mpc_ros::CommandsConfig& p_config, uint32_t p_level);

    /**
     * @brief      Disarms the motors
     */
    void disarmMotors();

    /**
     * @brief      Callback for the IMU messahes
     *
     * @param[in]  p_imu_msg  The imu message
     */
    void imuCallback(const sensor_msgs::ImuPtr& p_imu_msg);

    /**
     * @brief      Callback for the odometry message
     *
     * @param[in]  p_odometry_msg  The odometry message
     */
    void mavPoseCallback(const nav_msgs::OdometryConstPtr& p_odometry_msg);

    /**
     * @brief      Callback for the AABM path messages
     *
     * @param[in]  p_path_msg  The path message
     */
    void pathReferenceCallback(const mav_interface_msgs::Path& p_path_msg);

    /**
     * @brief      Processes the high level command
     *
     * @param[in]  p_hlv_command  The hlv command
     */
    void processHighLevelCommand(const HighLevelCmd p_hlv_command);

    /**
     * @brief      Publishes the control commands
     *
     * @param[in]  p_controlCmds  The control commands
     */
    void publishControlCmds(const controller::ControlCommands& p_controlCmds) const;

    bool sendAutopilotStatus(mav_interface_msgs::AutopilotStatusService::Request& request,
                             mav_interface_msgs::AutopilotStatusService::Response& response);

    /**
     * @brief      Publishes the reference position
     *
     * @param[in]  p_position_ref  The reference position
     */
    void publishPositionReference() const;

    /**
     * @brief      Callback for the AABM trajectory msgs
     *
     * @param[in]  p_trajectory_msg  The trajectory message
     */
    void
    trajectoryReferenceCallback(const mav_interface_msgs::FullStateTrajectory& p_trajectory_msg);

    const double m_Ts;                             ///< Discrete system timestep in seconds
    ros::NodeHandle m_nh;                          ///< Node handle
    ros::Subscriber m_mav_odometry_sub;            ///< Subscriber the MAV pose from okvis/Vicon
    ros::Subscriber m_imu_sub;                     ///< Subscriber for the IMU data
    ros::Subscriber m_bat_voltage_sub;             ///< Subscriber for the battery voltage
    ros::Subscriber m_pathCMD_sub;                 ///< Subscriber for the Path command
    ros::Subscriber m_trajectoryCMD_sub;           ///< Subscriber for the trajectory commands
    ros::Subscriber m_mavros_state_sub;            ///< Subscriber for the mavros state
    ros::Publisher m_control_pub;                  ///< Publisher for the control commands
    ros::Publisher m_ref_position_pub;             ///< Publisher for the reference position
    ros::Publisher m_atp_status_pub;               ///< Publisher for the autopilot status
    ros::Time m_heartbeatTimestamp;                ///< Last Heartbeat message timestamp
    controller::MPC m_mpc;                         ///< The MPC
    controller::MavState m_mav_state;              ///< The mavState
    std::shared_ptr<AutopilotQueue> m_atpQueuePtr; ///< Autopilot Queue
    std::shared_ptr<controller::MPCQueue> m_mpcQueuePtr; ///< MPC queue
    autopilot_state_machine::AutopilotFlags
        m_autopilotFlags; ///< Struct containing flags required for the state
                          ///< machine
    autopilot_state_machine::AutopilotStateMachine
        m_stateMachine; ///< (autopilot_state_machine::AutopilotStateMachine{m_autopilotFlags,
                        ///< m_atpQueuePtr, m_mpcQueuePtr});
    dynamic_reconfigure::Server<mpc_ros::CommandsConfig> m_rqtServer; ///< Dynamic reconfigure
    std::shared_ptr<controller::Px4ThrustModel>
        thrustModelPtr_;                         ///< Thrust Model for the PX4 autopilot
    bool m_publishControlCommands;               ///< When true, Commands are sent to the
                                                 ///< pixhawk
    ros::ServiceServer m_autopilotStatusService; ///< Server for resporting the
                                                 ///< autopilot status

    Parameters parameters_;

}; // class Autopilot

} // namespace autopilot
