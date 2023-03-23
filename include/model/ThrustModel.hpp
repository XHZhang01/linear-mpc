#pragma once

#include <mpc/Datatypes.hpp>
#include <yaml-cpp/yaml.h>

// Base Class for the Thrust Model
namespace controller {
class ThrustModel {
    public:
    /**
   * @brief      Constructs a new instance of the thrust model
   *
   * @param[in]  massNormalisedCoefficient  The mass normalised coefficient
   * @param[in]  totalMass                  The total MAV mass
   */
    ThrustModel(const double massNormalisedCoefficient, const double totalMass) :
            massNormalisedCoefficient_(massNormalisedCoefficient), totalMass_(totalMass){};

    /**
   * @brief      Destroys the object.
   */
    virtual ~ThrustModel(){};

    /**
   * @brief      Calculates the thrust in units (e.g. px4 uses a [0,1] value )
   * used from the attitude/low-level controller
   *
   * @param[in]  The inputAcceleration  The desired acceleration in g.
   *
   * @return     The computed scaled thrust to be forwarded to the low lever
   * controller.
   */
    virtual double computeScaledThrust(const double& inputAcceleration)
    {
        // Basic calculation used in e.g. the firefly and the px4
        const double scaledThrust = (inputAcceleration * massNormalisedCoefficient_ * totalMass_);
        return scaledThrust;
    }

    protected:
    const double massNormalisedCoefficient_; ///< Collective thrust coefficient.
    const double totalMass_;                 ///< The total mass of the drone with payload.
};

class Px4ThrustModel : ThrustModel {
    public:
    /**
   * @brief      Struct with the parameters used in the Px4ThrustModel
   */
    struct Parameters {
        double massNormalisedCoefficient = 0.5;
        double totalMass = 1.0;
        bool adaptiveThrust = false;
        std::array<double, 2> batteryModelCoeffs{0.0, 0.0};
        uint cells = 4;
        double filterCutoff = 10.0;

        Parameters(){};

        Parameters(const std::string filename)
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
                massNormalisedCoefficient =
                    config["px4thrust_model"]["mass_normalised_coeff"].as<double>();
                totalMass = config["px4thrust_model"]["total_mass"].as<double>();
                adaptiveThrust = config["px4thrust_model"]["adaptive_thrust"].as<bool>();
                std::vector<double> buffer =
                    config["px4thrust_model"]["battery_model_coeffs"].as<std::vector<double>>();
                std::copy(buffer.begin(), buffer.end(), batteryModelCoeffs.begin());
                std::cout << "Battery coeff " << batteryModelCoeffs[0] << "\t"
                          << batteryModelCoeffs[1] << std::endl;
                cells = config["px4thrust_model"]["cells"].as<uint>();
                filterCutoff = config["px4thrust_model"]["filter_cutoff_freq"].as<double>();
            }
            catch (YAML::Exception& e) {
                std::cout << "yaml Exception " << e.what() << std::endl;
                return;
            }
        }
    };

    /**
   * @brief      Constructs a new instance of the px4 thrust model
   *
   * @param[in]  massNormalisedCoefficient  The mass normalised coefficient
   * @param[in]  totalMass                  The total MAV mass
   * @param[in]  scaledThrustMin            The scaled thrust minimum
   * @param[in]  scaledThrustMax            The scaled thrust maximum
   * @param[in]  adaptiveThrust             The adaptive thrust
   * @param[in]  <unnamed>                  { parameter_description }
   */
    Px4ThrustModel(const double massNormalisedCoefficient,
                   const double totalMass,
                   const bool adaptiveThrust = false,
                   const std::array<double, 2> batteryModelCoeffs = {0.0, 0.0},
                   const uint cells = 4,
                   const double filterCutoff = 10.0) :
            ThrustModel(massNormalisedCoefficient, totalMass),
            adaptiveThrust_(adaptiveThrust),
            batteryModelCoeffs_(batteryModelCoeffs),
            cells_(cells),
            voltageLPF_(std::fabs(filterCutoff)),
            filteredVoltage_(cells_ * 4.2)
    {
    }

    /**
   * @brief      Constructs a new instance of the px4 thrust model given the
   * struct of parameters.
   *
   * @param[in]  parameters  The parameters
   */
    Px4ThrustModel(const Parameters& parameters) :
            ThrustModel(parameters.massNormalisedCoefficient, parameters.totalMass),
            adaptiveThrust_(parameters.adaptiveThrust),
            batteryModelCoeffs_(parameters.batteryModelCoeffs),
            cells_(parameters.cells),
            voltageLPF_(std::fabs(parameters.filterCutoff)),
            filteredVoltage_(cells_ * 4.2)
    {
    }

    /**
   * @brief      Calculates the scaled thrust.
   *
   * @param[in]  inputAcceleration  The input acceleration
   *
   * @return     The scaled thrust.
   */
    double computeScaledThrust(const double& inputAcceleration) override
    {
        if (!adaptiveThrust_) {
            const double scaledThrust =
                (inputAcceleration * massNormalisedCoefficient_ * totalMass_);

            return scaledThrust;
        }
        else {
            // Compute the coefficient based on the filtered voltage value

            // Clamp the filtered voltage to reasonable values.
            const double clampedVoltage =
                std::max(std::min(filteredVoltage_, cells_ * 4.2), cells_ * 3.2);

            const double thrustCoeff =
                batteryModelCoeffs_[0] * clampedVoltage + batteryModelCoeffs_[1];

            const double scaledThrust = (inputAcceleration * thrustCoeff * totalMass_);
            return scaledThrust;
        }
        return 0.0;
    }

    /**
   * @brief      Sets the voltage used in the thrust model
   *
   * @param[in]  inputVoltage  The input voltage
   * @param[in]  timestampSec  The timestamp security
   */
    void setVoltage(const double inputVoltage, const double timestampSec)
    {
        filteredVoltage_ = voltageLPF_.getFilteredValue(inputVoltage, timestampSec);
    }

    protected:
    const bool adaptiveThrust_;
    const std::array<double, 2> batteryModelCoeffs_;
    const uint cells_;

    LowPassFilter voltageLPF_;
    double filteredVoltage_;
};
} // namespace controller
