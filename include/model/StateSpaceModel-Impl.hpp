#pragma once
namespace controller {

template<size_t StateDim, size_t InputDim, size_t HorizonDim>
bool StateSpaceModel<StateDim, InputDim, HorizonDim>::c2d()
{
    // Check if matrices are initialized
    if (m_Ac.isZero(0) || m_Bc.isZero(0) || m_ts == 0.0)
        return false;

    // Compute the discrete time equivalent matrices
    Eigen::Matrix<double, StateDim + InputDim, StateDim + InputDim> K;

    // Copy m_Ac to the upper right block of K
    K.template block<StateDim, StateDim>(0, 0) = m_Ac;

    // Copy m_Bc to the upper left block of K
    K.template block<StateDim, InputDim>(0, StateDim) = m_Bc;

    // Fill in lower part with zeros
    K.template block<InputDim, StateDim + InputDim>(StateDim, 0) =
        Eigen::MatrixXd::Zero(InputDim, StateDim + InputDim);

    // Multiply  K with the sampling time
    K = K * m_ts;

    // Compute expm(K)
    Eigen::MatrixExponentialReturnValue<
        Eigen::Matrix<double, StateDim + InputDim, StateDim + InputDim>>
        K_exp(K);

    // Get the discrete time matrices from K_exp
    m_Ad = K_exp.template block<StateDim, StateDim>(0, 0);
    m_Bd = K_exp.template block<StateDim, InputDim>(0, StateDim);
    m_Cd = Eigen::MatrixXd::Identity(StateDim, StateDim);
    return true;
}

template<size_t StateDim, size_t InputDim, size_t HorizonDim>
Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim * InputDim>
StateSpaceModel<StateDim, InputDim, HorizonDim>::computeGamma() const
{
    // Implementation without loops
    Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim* StateDim> K =
        Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim * StateDim>::Zero();

    // Lower left part should be -kron(I(N-1),Ad);
    K.template block<(HorizonDim - 1) * StateDim, (HorizonDim - 1) * StateDim>(StateDim, 0) =
        Eigen::kroneckerProduct(
            Eigen::Matrix<double, (HorizonDim - 1), (HorizonDim - 1)>::Identity(), -m_Ad)
            .eval();
    K += Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim * StateDim>::Identity();

    return K.inverse()
        * Eigen::kroneckerProduct(Eigen::Matrix<double, HorizonDim, HorizonDim>::Identity(), m_Bd);
}

template<size_t StateDim, size_t InputDim, size_t HorizonDim>
Eigen::Matrix<double, HorizonDim * StateDim, StateDim>
StateSpaceModel<StateDim, InputDim, HorizonDim>::computePhi() const
{
    // Implementation without for loops
    Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim* StateDim> K =
        Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim * StateDim>::Zero();

    // Lower left part should be -kron(I(N-1),Ad);
    K.template block<(HorizonDim - 1) * StateDim, (HorizonDim - 1) * StateDim>(StateDim, 0) =
        Eigen::kroneckerProduct(
            Eigen::Matrix<double, (HorizonDim - 1), (HorizonDim - 1)>::Identity(), -m_Ad)
            .eval();
    K += Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim * StateDim>::Identity();

    // A tilda
    Eigen::Matrix<double, HorizonDim * StateDim, StateDim> A_tilda =
        Eigen::Matrix<double, HorizonDim * StateDim, StateDim>::Zero();
    A_tilda.template block<StateDim, StateDim>(0, 0) = m_Ad;

    return K.inverse() * A_tilda;
}
} // namespace controller