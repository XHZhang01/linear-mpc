#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>

namespace controller {

template<size_t StateDim, size_t InputDim, size_t HorizonDim = 1>
class StateSpaceModel {
    public:

    StateSpaceModel(const Eigen::Matrix<double, StateDim, StateDim>& p_Ac =
                        Eigen::Matrix<double, StateDim, StateDim>::Zero(),
                    const Eigen::Matrix<double, StateDim, InputDim>& p_Bc =
                        Eigen::Matrix<double, StateDim, InputDim>::Zero(),
                    const double p_ts = 0.0) :
            m_Ac(p_Ac),
            m_Bc(p_Bc),
            m_ts(p_ts),
            m_Ad(Eigen::Matrix<double, StateDim, StateDim>::Zero()),
            m_Bd(Eigen::Matrix<double, StateDim, InputDim>::Zero()),
            m_Cd(Eigen::Matrix<double, StateDim, StateDim>::Zero()){};

    ~StateSpaceModel(){};

    /**
   * @brief      Computes the Gamma matrix
   *
   * @return     The gamma matrix
   */
    Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim * InputDim> computeGamma() const;

    /**
   * @brief      Computes the phi matrix.
   *
   * @return     The phi matrix.
   */
    Eigen::Matrix<double, HorizonDim * StateDim, StateDim> computePhi() const;

    /**
   * @brief      Converts the continuous time system to an equivalent discrete
   * one using zoh discretization
   *
   * @return     Returns True if conversion is successful
   */
    bool c2d();

    /**
   * @brief      Gets the continuous time State transition matrix Ac.
   *
   * @return     The Ac matrix.
   */
    const Eigen::Matrix<double, StateDim, StateDim>& getAc() const
    {
        return m_Ac;
    }

    /**
   * @brief      Gets the discrete time State transition matrix Ad.
   *
   * @return     The Ad matrix.
   */
    const Eigen::Matrix<double, StateDim, StateDim>& getAd() const
    {
        return m_Ad;
    }

    /**
   * @brief      Gets the continuous time input matrix Bc.
   *
   * @return     The Bc matrix.
   */
    const Eigen::Matrix<double, StateDim, InputDim>& getBc() const
    {
        return m_Bc;
    }

    /**
   * @brief      Gets the discrete time input matrix Bd.
   *
   * @return     The Bd matrix.
   */
    const Eigen::Matrix<double, StateDim, InputDim>& getBd() const
    {
        return m_Bd;
    }

    /**
   * @brief      Gets the discrete time Cd matrix.
   *
   * @return     The Cd matrix.
   */
    const Eigen::Matrix<double, StateDim, StateDim>& getCd() const
    {
        return m_Cd;
    }

    /**
   * @brief      Gets the discretization time step Ts.

   *
   * @return     Discretization time step in seconds
   */
    double getTs() const
    {
        return m_ts;
    }

    /**
   * @brief      Sets the continuous time State transition matrix Ac.
   *
   * @param[in]  Ac    { continuous time State transition matrix Ac }
   */
    void setAc(const Eigen::Matrix<double, StateDim, StateDim>& Ac)
    {
        m_Ac = Ac;
    }

    /**
   * @brief      Sets the continuous time input matrix Bc.
   *
   * @param[in]  Bc    { the continuous time input matrix Bc }
   */
    void setBc(const Eigen::Matrix<double, StateDim, InputDim>& Bc)
    {
        m_Bc = Bc;
    }

    /**
   * @brief      Sets the discretization time step Ts.
   *
   * @param[in]  Ts    Discretization time step in seconds
   */
    void setTs(double ts)
    {
        m_ts = ts;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    Eigen::Matrix<double, StateDim, StateDim> m_Ac; ///< Continuous time state transition matrix
    Eigen::Matrix<double, StateDim, InputDim> m_Bc; ///< Continuous time input matrix
    double m_ts;                                    ///< Sampling time in seconds

    Eigen::Matrix<double, StateDim, StateDim> m_Ad; ///< Discrete time state transition matrix
    Eigen::Matrix<double, StateDim, InputDim> m_Bd; ///< Discrete time input matrix
    Eigen::Matrix<double, StateDim, StateDim> m_Cd; ///< Discrete time output matrix
};

} // namespace controller

#include <model/StateSpaceModel-Impl.hpp>
