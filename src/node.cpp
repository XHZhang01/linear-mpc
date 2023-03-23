/* Copyright (C) Imperial College, Smart Robotics Lab - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Dimos Tzoumanikas <dt214@ic.ac.uk>
 * 2015-2019
 */

#include <atp/Autopilot.hpp>

int main(int argc, char** argv)
{
    // Convenience.
    using ThrustModel = controller::Px4ThrustModel;
    using MavModel = controller::FirstOrderModel<controller::kHorizonLength>;
    using AutopilotParameters = autopilot::Autopilot::Parameters;

    ros::init(argc, argv, "autopilot");

    ros::NodeHandle n;
    std::string config_file_path_autopilot;

    // Make sure the config file exists.
    if (n.getParam("config_file_autopilot", config_file_path_autopilot)) {
    }
    else {
        std::cout << "Config file for autopilot parameters not found" << std::endl;
        return -1;
    }

    // Read Linear MAV model parameters, gains and create model
    const MavModel::Parameters modelParameters(config_file_path_autopilot);
    const MavModel::Gains modelGains(config_file_path_autopilot);
    const MavModel::Constraints boxConstraints(config_file_path_autopilot);
    MavModel mavModel(modelParameters, modelGains, boxConstraints);

    // Read thrust model parameters and create object
    const ThrustModel::Parameters thrustModelParams(config_file_path_autopilot);
    std::shared_ptr<ThrustModel> thrustModelPtr(new ThrustModel(thrustModelParams));

    // Read the autopilot parameters
    const AutopilotParameters autopilotParameters(config_file_path_autopilot);

    // Set up MPC Queue
    std::shared_ptr<controller::MPCQueue> mpcQueuePtr(new controller::MPCQueue());

    // Set up Autopilot Queue
    std::shared_ptr<autopilot::AutopilotQueue> queueAutopilotPtr(new autopilot::AutopilotQueue());

    // Bring up the autopilot
    autopilot::Autopilot dummyPilot(
        n, autopilotParameters, mavModel, queueAutopilotPtr, mpcQueuePtr, thrustModelPtr);

    // ROS loop
    ros::spin();
    return 0;
}
