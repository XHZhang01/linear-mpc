#pragma once

#include <model/StateSpaceModel.hpp>
#include <mpc/Helper.hpp>
#include <yaml-cpp/yaml.h>

namespace controller {

template<size_t StateDim, size_t InputDim, size_t HorizonLength>
class MavModel : public StateSpaceModel<StateDim, InputDim, HorizonLength> {
    public:
    using StateVector = Eigen::Matrix<double, StateDim, 1>;
    using InputVector = Eigen::Matrix<double, InputDim, 1>;

    using TrackingStateGains = Eigen::Matrix<double, StateDim, StateDim>;
    using TrackingInputGains = Eigen::Matrix<double, InputDim, InputDim>;

    MavModel(){};

    const size_t kXIdx_ = 0;
    const size_t kYIdx_ = 3;
    const size_t kZIdx_ = 6;

    virtual StateVector buildStateVector(const Eigen::Vector3d r_NB,
                                         const Eigen::Vector3d v_NB,
                                         const double theta,
                                         const double phi) const = 0;

    virtual TrackingStateGains getTrackingStateGains() const = 0;

    virtual TrackingInputGains getTrackingInputGains() const = 0;

    virtual InputVector getBoxConstraintsMin() const = 0;

    virtual InputVector getBoxConstraintsMax() const = 0;

    virtual ~MavModel(){};

    private:
};

template<size_t HorizonLength = 1>
class FirstOrderModel : public MavModel<8, 3, HorizonLength> {
    public:

    using StateVector = typename MavModel<8, 3, HorizonLength>::StateVector;
    using InputVector = typename MavModel<8, 3, HorizonLength>::InputVector;
    using TrackingStateGains = typename MavModel<8, 3, HorizonLength>::TrackingStateGains;
    using TrackingInputGains = typename MavModel<8, 3, HorizonLength>::TrackingInputGains;

    // State vector consists of [x,xdot,theta,y,ydot,phi,z,zdot] in the order
    // given bellow.
    //
    const size_t kXIdx_ = 0;
    const size_t kXdotIdx_ = 1;
    const size_t kThetaIdx_ = 2;
    const size_t kYIdx_ = 3;
    const size_t kYdotIdx_ = 4;
    const size_t kPhiIdx_ = 5;
    const size_t kZIdx_ = 6;
    const size_t kZdotIdx_ = 7;

    // Input vector consists of [theta_ref,phi_ref,thrust_ref] in the order given
    // bellow.
    const size_t kUThetaIdx_ = 0;
    const size_t kUPhiIdx_ = 1;
    const size_t kUThrustIdx_ = 2;

    /**
   * @brief      Struct containing the gain parameters
   */
    struct Gains {
        // State tracking gains
        double x_gain = 7.0;
        double y_gain = 7.3;
        double z_gain = 7.0;

        double xDot_gain = 1.0;
        double yDot_gain = 1.0;
        double zDot_gain = 1.5;

        double theta_gain = 0.1;
        double phi_gain = 0.1;

        // Control input gains.
        double theta_ref_gain = 2.0;
        double phi_ref_gain = 2.0;
        double thrust_ref_gain = 3.1;

        Gains() = delete;

        Gains(const std::string& filename)
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

            try {
                // Position
                x_gain = config["control_gains"]["x_gain"].as<double>();
                y_gain = config["control_gains"]["y_gain"].as<double>();
                z_gain = config["control_gains"]["z_gain"].as<double>();

                xDot_gain = config["control_gains"]["x_dot_gain"].as<double>();
                yDot_gain = config["control_gains"]["y_dot_gain"].as<double>();
                zDot_gain = config["control_gains"]["z_dot_gain"].as<double>();

                theta_gain = config["control_gains"]["theta_gain"].as<double>();
                phi_gain = config["control_gains"]["phi_gain"].as<double>();

                theta_ref_gain = config["control_gains"]["theta_ref_gain"].as<double>();
                phi_ref_gain = config["control_gains"]["phi_ref_gain"].as<double>();
                thrust_ref_gain = config["control_gains"]["thrust_ref_gain"].as<double>();
            }
            catch (YAML::Exception& e) {
                std::cout << "yaml Exception " << e.what() << std::endl;
                return;
            }

            // Print parameters
        }
    };

    struct Constraints {
        double theta_min = controller::deg2rad(-20.0);
        double theta_max = controller::deg2rad(20.0);

        double phi_min = controller::deg2rad(-20.0);
        double phi_max = controller::deg2rad(20.0);

        double thrust_min = -0.8;
        double thrust_max = 0.8;

        Constraints() = delete;

        Constraints(const std::string& filename)
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

            try {
                // Position
                theta_min =
                    controller::deg2rad(config["box_constraints"]["theta_min"].as<double>());
                theta_max =
                    controller::deg2rad(config["box_constraints"]["theta_max"].as<double>());

                phi_min = controller::deg2rad(config["box_constraints"]["phi_min"].as<double>());
                phi_max = controller::deg2rad(config["box_constraints"]["phi_max"].as<double>());

                thrust_min = config["box_constraints"]["thrust_min"].as<double>();
                thrust_max = config["box_constraints"]["thrust_max"].as<double>();
            }
            catch (YAML::Exception& e) {
                std::cout << "yaml Exception " << e.what() << std::endl;
                return;
            }

            // Print Constraints
        }
    };

    /**
   * @brief      Parameters of the closed loop attitude/translational dynamics
   * subsystems
   */
    struct Parameters {
        double Ts = 0.025;
        double theta_dc_gain = 1.0;
        double theta_tau = 0.1;
        double phi_dc_gain = 1.0;
        double phi_tau = 0.1;

        double x_drag_coeff = 0.01;
        double y_drag_coeff = 0.01;
        double z_drag_coeff = 0.01;

        Parameters() = delete;

        Parameters(const std::string& filename)
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
            try {
                // Read Parameters
                Ts = config["autopilot_parameters"]["Ts"].as<double>();
                theta_dc_gain = config["mav_model"]["theta_dc_gain"].as<double>();
                theta_tau = config["mav_model"]["theta_tau"].as<double>();
                phi_dc_gain = config["mav_model"]["phi_dc_gain"].as<double>();
                phi_tau = config["mav_model"]["phi_tau"].as<double>();
                x_drag_coeff = config["mav_model"]["x_drag_coeff"].as<double>();
                y_drag_coeff = config["mav_model"]["y_drag_coeff"].as<double>();
                z_drag_coeff = config["mav_model"]["z_drag_coeff"].as<double>();
            }
            catch (YAML::Exception& e) {
                std::cout << "yaml Exception " << e.what() << std::endl;
                return;
            }
        }
    };

    FirstOrderModel() = delete;

    /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  parameters  The model parameters
   * @param[in]  gains       The gains used for control
   */
    FirstOrderModel(const Parameters& parameters,
                    const Gains& gains,
                    const Constraints& constraints) :
            gains_(gains), parameters_(parameters), constraints_(constraints)
    {
        // Construct the continuous time state trantition matrix x_dot = Ax + Bu
        Eigen::Matrix<double, 8, 8> Ac = Eigen::Matrix<double, 8, 8>::Zero();

        // Construct the state transition matrix
        Ac(kXIdx_, kXdotIdx_) = 1.0;
        Ac(kXdotIdx_, kXdotIdx_) = -parameters_.x_drag_coeff;
        Ac(kXdotIdx_, kThetaIdx_) = 9.81;
        Ac(kThetaIdx_, kThetaIdx_) = -(1. / parameters_.theta_tau);

        Ac(kYIdx_, kYdotIdx_) = 1.0;
        Ac(kYdotIdx_, kYdotIdx_) = -parameters_.y_drag_coeff;
        Ac(kYdotIdx_, kPhiIdx_) = -9.81;
        Ac(kPhiIdx_, kPhiIdx_) = -(1. / parameters_.phi_tau);

        Ac(kZIdx_, kZdotIdx_) = 1.0;
        Ac(kZdotIdx_, kZdotIdx_) = -parameters_.z_drag_coeff;

        // Construct input transition matrix.
        Eigen::Matrix<double, 8, 3> Bc = Eigen::Matrix<double, 8, 3>::Zero();

        Bc(kThetaIdx_, kUThetaIdx_) = parameters_.theta_dc_gain / parameters_.theta_tau;
        Bc(kPhiIdx_, kUPhiIdx_) = parameters_.phi_dc_gain / parameters_.phi_tau;
        Bc(kZdotIdx_, kUThrustIdx_) = 1.0;

        // Pass this to the state space model.
        this->setAc(Ac);
        this->setBc(Bc);
        this->setTs(parameters_.Ts);
    }

    /**
   * @brief      Builds a state vector given the position, velocity and roll
   * pitch data
   *
   * @param[in]  r_NB   The position in the N frame
   * @param[in]  v_NB   The velocity in the N frame
   * @param[in]  theta  The theta angle in radians
   * @param[in]  phi    The phi angle in radians
   *
   * @return     The state vector.
   */
    StateVector buildStateVector(const Eigen::Vector3d r_NB,
                                 const Eigen::Vector3d v_NB,
                                 const double theta,
                                 const double phi) const override
    {
        StateVector state = StateVector::Zero();
        state(kXIdx_) = r_NB(0);    // X
        state(kXdotIdx_) = v_NB(0); // X dot
        state(kThetaIdx_) = theta;  // theta

        state(kYIdx_) = r_NB(1);    // Y
        state(kYdotIdx_) = v_NB(1); // Y dot
        state(kPhiIdx_) = phi;      // phi

        state(kZIdx_) = r_NB(2);    // Z
        state(kZdotIdx_) = v_NB(2); // Z dot

        return state;
    }

    /**
   * @brief      Returns the tracking state gains.
   *
   * @return     The tracking state gains.
   */
    TrackingStateGains getTrackingStateGains() const override
    {
        TrackingStateGains stateGains = TrackingStateGains::Zero();

        // Fill in the x subsystem gains
        stateGains(kXIdx_, kXIdx_) = gains_.x_gain;
        stateGains(kXdotIdx_, kXdotIdx_) = gains_.xDot_gain;
        stateGains(kThetaIdx_, kThetaIdx_) = gains_.theta_gain;

        // Fill in the y subsystem gains
        stateGains(kYIdx_, kYIdx_) = gains_.y_gain;
        stateGains(kYdotIdx_, kYdotIdx_) = gains_.yDot_gain;
        stateGains(kPhiIdx_, kPhiIdx_) = gains_.phi_gain;

        // Fill in the z subsystem gains
        stateGains(kZIdx_, kZIdx_) = gains_.z_gain;
        stateGains(kZdotIdx_, kZdotIdx_) = gains_.zDot_gain;

        return stateGains;
    }

    /**
   * @brief      Returns the tracking input
   *
   * @return     The tracking input gains.
   */
    TrackingInputGains getTrackingInputGains() const override
    {
        TrackingInputGains inputGains = TrackingInputGains::Zero();

        // Fill in the x subsystem gains
        inputGains(kUThetaIdx_, kUThetaIdx_) = gains_.theta_ref_gain;
        inputGains(kUPhiIdx_, kUPhiIdx_) = gains_.phi_ref_gain;
        inputGains(kUThrustIdx_, kUThrustIdx_) = gains_.thrust_ref_gain;

        return inputGains;
    }

    /**
   * @brief      Returns the lower bound of the box constraints.
   *
   * @return     The lower Bound of the box constraints.
   */
    InputVector getBoxConstraintsMin() const override
    {
        InputVector uMin = InputVector::Zero();
        uMin(kUThetaIdx_) = constraints_.theta_min;
        uMin(kUPhiIdx_) = constraints_.phi_min;
        uMin(kUThrustIdx_) = constraints_.thrust_min;
        return uMin;
    }

    /**
   * @brief      Returns the upper bound of the box constraints.
   *
   * @return     The box constraints maximum.
   */
    InputVector getBoxConstraintsMax() const override
    {
        InputVector uMax = InputVector::Zero();
        uMax(kUThetaIdx_) = constraints_.theta_max;
        uMax(kUPhiIdx_) = constraints_.phi_max;
        uMax(kUThrustIdx_) = constraints_.thrust_max;
        return uMax;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    const Gains gains_;
    const Parameters parameters_;
    const Constraints constraints_;
};

} // namespace controller
