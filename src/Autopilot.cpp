#include <atp/Autopilot.hpp>
#include <cassert>
#include <fstream>
#include <iostream>

namespace autopilot {

Autopilot::Parameters::Parameters(const std::string filename)
{
    YAML::Node config;
    // Open the YAML file
    try {
        config = YAML::LoadFile(filename);
    }
    catch (YAML::Exception& e) {
        std::cout << " Problem opening Yaml File" << std::endl;
        std::cout << e.what() << std::endl;
        return;
    }

    // Read Parameters
    try {
        const YAML::Node& autopilot_config = config["autopilot_parameters"];

        take_off_height = autopilot_config["take_off_height"].as<double>();
        take_off_velocity = std::fabs(autopilot_config["take_off_velocity"].as<double>());
        landing_height = autopilot_config["landing_height"].as<double>();
        landing_velocity = std::fabs(autopilot_config["landing_velocity"].as<double>());

        const size_t n = std::min(autopilot_config["home_position"].size(),
                                  static_cast<size_t>(home_position.size()));
        for (size_t i = 0; i < n; i++) {
            home_position[i] = autopilot_config["home_position"][i].as<double>();
        }

        return_home_height = autopilot_config["return_home_height"].as<double>();
        return_home_velocity = autopilot_config["return_home_velocity"].as<double>();
    }
    catch (YAML::Exception& e) {
        std::cout << " Problem opening Yaml File" << std::endl;
        std::cout << e.what() << std::endl;
        return;
    }
}

Autopilot::Autopilot(
    ros::NodeHandle& p_nh,
    const Parameters& parameters,
    const controller::FirstOrderModel<controller::kHorizonLength>& p_state_space_model,
    const std::shared_ptr<AutopilotQueue>& p_atpQueuePtr,
    const std::shared_ptr<controller::MPCQueue>& p_mpcQueuePtr,
    const std::shared_ptr<controller::Px4ThrustModel>& thrustModelPtr) :
        m_Ts(p_state_space_model.getTs()),
        m_nh(p_nh),
        m_mav_odometry_sub(m_nh.subscribe("mavros/odometry/data",
                                          1,
                                          &Autopilot::mavPoseCallback,
                                          this,
                                          ros::TransportHints().tcpNoDelay())),
        m_imu_sub(m_nh.subscribe("mavros/imu/data",
                                 1,
                                 &Autopilot::imuCallback,
                                 this,
                                 ros::TransportHints().tcpNoDelay())),
        m_bat_voltage_sub(
            m_nh.subscribe("mavros/battery", 1, &Autopilot::batteryStateCallback, this)),
        m_pathCMD_sub(
            m_nh.subscribe("autopilot/PathReference", 30, &Autopilot::pathReferenceCallback, this)),
        m_trajectoryCMD_sub(
            m_nh.subscribe("/supereight/path", 30, &Autopilot::trajectoryReferenceCallback, this)),
        m_control_pub(
            m_nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1)),
        m_ref_position_pub(
            m_nh.advertise<geometry_msgs::PointStamped>("autopilot/PositionReference", 1)),
        m_heartbeatTimestamp(ros::Time::now()),
        m_mpc(p_state_space_model, p_mpcQueuePtr),
        m_mav_state(controller::MavState()),
        m_atpQueuePtr(p_atpQueuePtr),
        m_mpcQueuePtr(p_mpcQueuePtr),
        m_autopilotFlags(autopilot_state_machine::AutopilotFlags()),
        m_stateMachine(autopilot_state_machine::AutopilotStateMachine{m_autopilotFlags,
                                                                      m_mpcQueuePtr,
                                                                      m_atpQueuePtr}),
        thrustModelPtr_(thrustModelPtr),
        m_publishControlCommands(true),
        m_autopilotStatusService(m_nh.advertiseService("autopilot/statusService",
                                                       &Autopilot::sendAutopilotStatus,
                                                       this)),
        parameters_(parameters)
{
    // Dynamic reconfigure
    m_rqtServer.setCallback(boost::bind(&Autopilot::dynamicReconfigureCallback, this, _1, _2));
}

void Autopilot::batteryStateCallback(const sensor_msgs::BatteryStatePtr& p_batteryStateMsg)
{
    // m_mpc.setBatteryVoltage(static_cast<double>(p_batteryStateMsg->voltage),
    // p_batteryStateMsg->header.stamp.toSec());
    //
    // set the voltage to the model.
    thrustModelPtr_->setVoltage(static_cast<double>(p_batteryStateMsg->voltage),
                                p_batteryStateMsg->header.stamp.toSec());
}

void Autopilot::disarmMotors()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    m_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming").call(arm_cmd);
    if (arm_cmd.response.success) {
        m_autopilotFlags.motorsArmed = false;

        // Process event Disarm
        m_stateMachine.process_event(autopilot_state_machine::DisarmCommand{});
    }
}

void Autopilot::dynamicReconfigureCallback(mpc_ros::CommandsConfig& p_config, uint32_t)
{
    if (p_config.land_home) {
        // Process return home command
        p_config.land_home = false;
        processHighLevelCommand(HighLevelCmd::kReturn2home);
    }

    if (p_config.land) {
        // Process land command
        p_config.land = false;
        processHighLevelCommand(HighLevelCmd::kLand);
    }

    if (p_config.take_off) {
        // Process take off command
        p_config.take_off = false;
        processHighLevelCommand(HighLevelCmd::kTakeOff);
    }
}

void Autopilot::imuCallback(const sensor_msgs::ImuPtr& p_imu_msg)
{
    // Update the IMU data required for control
    m_autopilotFlags.receivedImuMsg = true;
    m_mpc.setAttitudeControlFrame(Eigen::Quaterniond(p_imu_msg->orientation.w,
                                                     p_imu_msg->orientation.x,
                                                     p_imu_msg->orientation.y,
                                                     p_imu_msg->orientation.z)
                                      .normalized());
}

void Autopilot::mavPoseCallback(const nav_msgs::OdometryConstPtr& p_odometry_msg)
{
    m_autopilotFlags.receivedOdometryMsg = true;

    // Update
    m_mav_state = MavState(
        p_odometry_msg, controller::ReferenceFrame::kWorld, controller::ReferenceFrame::kBody);

    // Process New odometry event
    m_stateMachine.process_event(full_pose_state_machine::odometryMessageEvent{m_mav_state});

    // Send a take Off command
    static bool runOnce = true;
    if (runOnce) {
        // Send a take off.
        processHighLevelCommand(HighLevelCmd::kTakeOff);
        runOnce = false;
    }

    // Check state machine and run the controller if required
    if (m_stateMachine.is(autopilot_state_machine::SML_fullPoseControl)) {
        // Pass MavState to the controller
        m_mpc.setMavState(m_mav_state);

        // Run the controller
        controller::ControlCommands mpcCommand;
        const bool converged = m_mpc.computeControl(mpcCommand);

        // Publish Command
        if (converged)
            publishControlCmds(mpcCommand);
    }
    else {
        // Publish a zero thrust command. Required for the transition to offboard
        // mode
        const controller::ControlCommands zeroThrustCommand(
            controller::Timestamp(0, 0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), 0.0);
        publishControlCmds(zeroThrustCommand);
    }

    // When in trajectory mode, trigger a check MPC queue event
    m_stateMachine.process_event(full_pose_state_machine::checkMPCQueueEvent{});

    // Output a status message
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "Autopilot Tasks->\t" << m_atpQueuePtr->size()
                                                   << "\tMPC Queue size->\t"
                                                   << m_mpcQueuePtr->size());

    // Output a warning if the Callback is triggered in a different rate than the
    // Discretization step
    /*
   */

    // Publish Position Reference
    publishPositionReference();
}

void Autopilot::pathReferenceCallback(const mav_interface_msgs::Path& p_path_msg)
{
    // Check that the state machine is not in Idle mode.
    if (m_stateMachine.is(autopilot_state_machine::SML_idleState)) {
        return;
    }

    // Convert path msg to autopilot::path
    const Path path(p_path_msg);

    // Check if we need to flush the reference Queue.
    if (path.flushQueue && path.waypoints.size()) {
        m_stateMachine.process_event(full_pose_state_machine::FlushQueueEvent{});
    }

    // Push all the Waypoints to the Queue.
    for (const auto waypoint : path.waypoints) {
        m_atpQueuePtr->push_back(waypoint);
    }

    // Trigger new reference event
    m_stateMachine.process_event(full_pose_state_machine::newReferenceEvent{});
}

void Autopilot::processHighLevelCommand(const HighLevelCmd p_hlv_command)
{
    switch (p_hlv_command) {
    case HighLevelCmd::kTakeOff: {
        // ToDo -> Ignore if the drone is flying...

        // Take Off trajectory //-> Set the take off position bellow the current one
        // to avoid drift due to the ground effect
        Trajectory takeOffTrajectory;
        generateStraightTrajectory(m_mav_state.position - Eigen::Vector3d(0.0, 0.0, 0.5),
                                   Eigen::Vector3d(m_mav_state.position(0),
                                                   m_mav_state.position(1),
                                                   parameters_.take_off_height),
                                   parameters_.take_off_velocity,
                                   controller::quat2yaw(m_mav_state.orientation),
                                   2.0,
                                   M_PI,
                                   1e+9 * m_Ts,
                                   takeOffTrajectory);

        // Push Initial Waypoint
        m_atpQueuePtr->push_back(takeOffTrajectory.initialWaypoint);

        // Push Trajectory part
        m_atpQueuePtr->push_back(takeOffTrajectory);

        // Trigger new reference event
        m_stateMachine.process_event(full_pose_state_machine::newReferenceEvent{});

        break;
    }
    case HighLevelCmd::kLand: {
        // Clear Queue
        m_stateMachine.process_event(full_pose_state_machine::FlushQueueEvent{});

        // Land trajectory
        Trajectory landTrajectory;
        generateStraightTrajectory(m_mav_state.position,
                                   Eigen::Vector3d(m_mav_state.position(0),
                                                   m_mav_state.position(1),
                                                   parameters_.landing_height),
                                   parameters_.landing_velocity,
                                   controller::quat2yaw(m_mav_state.orientation),
                                   0.5,
                                   M_PI,
                                   1e+9 * m_Ts,
                                   landTrajectory);

        // Push Initial Waypoint
        m_atpQueuePtr->push_back(landTrajectory.initialWaypoint);

        // Push Trajectory part
        m_atpQueuePtr->push_back(landTrajectory);

        // Trigger new reference event
        m_stateMachine.process_event(full_pose_state_machine::newReferenceEvent{});
        break;
    }
    case HighLevelCmd::kReturn2home: {
        // Clear Queue
        m_stateMachine.process_event(full_pose_state_machine::FlushQueueEvent{});

        // Fly to the desired height.

        // Go straight up trajectory
        Trajectory goUpTrajectory;
        generateStraightTrajectory(m_mav_state.position,
                                   Eigen::Vector3d(m_mav_state.position.x(),
                                                   m_mav_state.position.y(),
                                                   parameters_.return_home_height),
                                   parameters_.return_home_velocity,
                                   controller::quat2yaw(m_mav_state.orientation),
                                   0.5,
                                   M_PI,
                                   1e+9 * m_Ts,
                                   goUpTrajectory);

        // Return 2 home trajectory
        Trajectory returnHomeTrajectory;
        generateStraightTrajectory(Eigen::Vector3d(m_mav_state.position.x(),
                                                   m_mav_state.position.y(),
                                                   parameters_.return_home_height),
                                   Eigen::Vector3d(parameters_.home_position.x(),
                                                   parameters_.home_position.y(),
                                                   parameters_.return_home_height),
                                   parameters_.return_home_velocity,
                                   controller::quat2yaw(m_mav_state.orientation),
                                   0.5,
                                   M_PI,
                                   1e+9 * m_Ts,
                                   returnHomeTrajectory);

        // Land home trajectory
        Trajectory landHomeTrajectory;
        generateStraightTrajectory(Eigen::Vector3d(parameters_.home_position.x(),
                                                   parameters_.home_position.y(),
                                                   parameters_.return_home_height),
                                   parameters_.home_position,
                                   parameters_.return_home_velocity,
                                   controller::quat2yaw(m_mav_state.orientation),
                                   0.5,
                                   M_PI,
                                   1e+9 * m_Ts,
                                   landHomeTrajectory);

        // Push Initial Waypoint - Go up
        m_atpQueuePtr->push_back(goUpTrajectory.initialWaypoint);

        // Push Trajectory path -  Go up
        m_atpQueuePtr->push_back(goUpTrajectory);

        // Push Initial Waypoint - Return home
        m_atpQueuePtr->push_back(returnHomeTrajectory.initialWaypoint);

        // Push Trajectory path -  Return home
        m_atpQueuePtr->push_back(returnHomeTrajectory);

        // Push Initial Waypoint - Land Home
        m_atpQueuePtr->push_back(landHomeTrajectory.initialWaypoint);

        // Push Trajectory part - Land Home
        m_atpQueuePtr->push_back(landHomeTrajectory);

        // Trigger new reference event
        m_stateMachine.process_event(full_pose_state_machine::newReferenceEvent{});
        break;
    }

    case HighLevelCmd::kDisarm: {
        // Arm the motors
        disarmMotors();
        break;
    }
    default: {
        // Do nothing
        break;
    }
    }
}

bool Autopilot::sendAutopilotStatus(mav_interface_msgs::AutopilotStatusService::Request&,
                                    mav_interface_msgs::AutopilotStatusService::Response& response)
{
    // Check if the state machine is in the Full pose mode.
    if (m_stateMachine.is(autopilot_state_machine::SML_fullPoseControl)) {
        // Autopilot is up and running
        response.status.initialised = true;

        // Fill in the mode
        if (m_stateMachine.is<decltype(
                boost::sml::state<full_pose_state_machine::fullPoseControlStateMachine>)>(
                full_pose_state_machine::SML_idleState)) {
            // Idle mode
            response.status.taskType = response.status.Idle;
        }
        else if (m_stateMachine.is<decltype(
                     boost::sml::state<full_pose_state_machine::fullPoseControlStateMachine>)>(
                     full_pose_state_machine::SML_waypointState)) {
            // Waypoint mode
            response.status.taskType = response.status.Waypoint;
        }
        else if (m_stateMachine.is<decltype(
                     boost::sml::state<full_pose_state_machine::fullPoseControlStateMachine>)>(
                     full_pose_state_machine::SML_trajectoryState)) {
            // Trajectory mode
            response.status.taskType = response.status.Trajectory;
        }

        // ToDo-> get ID of the current task.
    }
    else {
        // Controller not running.
        response.status.initialised = false;
    }

    // Fill in header
    response.status.header.stamp = ros::Time::now();
    return true;
}

void Autopilot::publishControlCmds(const controller::ControlCommands& p_controlCmds) const
{
    mavros_msgs::AttitudeTarget attitudeCmd;

    // Fill current timestamp
    attitudeCmd.header.stamp = ros::Time::now();

    // Fill control mask -> send orientation command
    attitudeCmd.type_mask =
        attitudeCmd.IGNORE_ROLL_RATE | attitudeCmd.IGNORE_PITCH_RATE | attitudeCmd.IGNORE_YAW_RATE;

    // Fill reference quaternion.
    const Eigen::Quaterniond referenceOrientation = p_controlCmds.orientation.normalized();
    attitudeCmd.orientation.w = referenceOrientation.w();
    attitudeCmd.orientation.x = referenceOrientation.x();
    attitudeCmd.orientation.y = referenceOrientation.y();
    attitudeCmd.orientation.z = referenceOrientation.z();

    // Rates values are ignored by PX4. Set zero anyway.
    attitudeCmd.body_rate.x = 0.0;
    attitudeCmd.body_rate.y = 0.0;
    attitudeCmd.body_rate.z = 0.0;

    // Compute FF thrust, required for hover. // ToDo -> clamp the FF term.
    const double scaledThrustFF = thrustModelPtr_->computeScaledThrust(1.0);

    // Scale the feedback term provided by the controller.
    const double scaledThrust = thrustModelPtr_->computeScaledThrust(p_controlCmds.thrust);

    // Fill in reference thrust.
    attitudeCmd.thrust = std::max(std::min(scaledThrustFF + scaledThrust, 1.0), 0.0);

    // Publish control message
    m_control_pub.publish(attitudeCmd);
}

void Autopilot::publishPositionReference() const
{
    // Get the Reference Position from the MPC
    const Eigen::Vector3d p_position_ref = m_mpc.getPositionReference();

    geometry_msgs::PointStamped refPosition;
    refPosition.header.stamp = ros::Time::now();
    refPosition.point.x = p_position_ref(0);
    refPosition.point.y = p_position_ref(1);
    refPosition.point.z = p_position_ref(2);
    m_ref_position_pub.publish(refPosition);
}

void Autopilot::trajectoryReferenceCallback(
    const mav_interface_msgs::FullStateTrajectory& p_trajectory_msg)
{
    // Accept references when not in Idle
    if (m_stateMachine.is(autopilot_state_machine::SML_idleState)) {
        return;
    }

    // Convert trajectory Msg to autopilot::Trajectory
    const Trajectory trajectory(p_trajectory_msg);

    // Resample and push if the input Trajectory is valid
    Trajectory resampledTrajectory;
    if (resampleTrajectory(trajectory, 1e+9 * m_Ts, resampledTrajectory)) {
        // Check if we need to flush the reference Queue.
        if (resampledTrajectory.flushQueue) {
            m_stateMachine.process_event(full_pose_state_machine::FlushQueueEvent{});
        }

        // Push initial waypoint to the Queue
        m_atpQueuePtr->push_back(resampledTrajectory.initialWaypoint);

        // Push trajectory part
        m_atpQueuePtr->push_back(resampledTrajectory);

        // Trigger new reference event
        m_stateMachine.process_event(full_pose_state_machine::newReferenceEvent{});
    }

} // namespace autopilot

} // namespace autopilot
