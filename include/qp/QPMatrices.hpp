#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace controller {
/**
 * @brief      Struct with the Matrices used when the MPC problem is formulated
 * as a QP.
 */
template<size_t StateDim, size_t InputDim, size_t HorizonDim>
struct QPMatrices {
    // Quadratic cost term used in the setpoint case.
    Eigen::Matrix<double, HorizonDim * InputDim, HorizonDim* InputDim> Q_s =
        Eigen::MatrixXd::Zero(HorizonDim * InputDim, HorizonDim* InputDim);

    // Quadratic cost term used in the trajectory case.
    Eigen::Matrix<double, HorizonDim * InputDim, HorizonDim* InputDim> Q_t =
        Eigen::MatrixXd::Zero(HorizonDim * InputDim, HorizonDim* InputDim);

    // Linear cost term
    Eigen::Matrix<double, HorizonDim * InputDim, 1> C =
        Eigen::MatrixXd::Zero(HorizonDim * InputDim, 1);

    // Box constraints
    Eigen::Matrix<double, InputDim * HorizonDim, 1> u_max =
        Eigen::MatrixXd::Zero(InputDim * HorizonDim, 1);

    // Box constraints
    Eigen::Matrix<double, InputDim * HorizonDim, 1> u_min =
        Eigen::MatrixXd::Zero(InputDim * HorizonDim, 1);

    // C_bar
    Eigen::Matrix<double, HorizonDim * StateDim, HorizonDim* StateDim> C_bar =
        Eigen::MatrixXd::Zero(HorizonDim * StateDim, HorizonDim* StateDim);

    // R_bar -> R_bar containing the setpoint gains
    Eigen::Matrix<double, HorizonDim * InputDim, HorizonDim* InputDim> R_bar_s =
        Eigen::MatrixXd::Zero(HorizonDim * InputDim, HorizonDim* InputDim);

    // R_bar -> R_bar containing the trajectory gains. This was for the AABM
    // experiments with the idea of having tigher position gains for the
    // trajectories.
    Eigen::Matrix<double, HorizonDim * InputDim, HorizonDim* InputDim> R_bar_t =
        Eigen::MatrixXd::Zero(HorizonDim * InputDim, HorizonDim* InputDim);

    // A_qp for the setpoint case
    Eigen::Matrix<double, HorizonDim*(StateDim + InputDim), HorizonDim* InputDim> A_qp_s =
        Eigen::MatrixXd::Zero(HorizonDim * (StateDim + InputDim), HorizonDim* InputDim);

    // A_qp for the trajectory case
    Eigen::Matrix<double, HorizonDim*(StateDim + InputDim), HorizonDim* InputDim> A_qp_t =
        Eigen::MatrixXd::Zero(HorizonDim * (StateDim + InputDim), HorizonDim* InputDim);
    //
    Eigen::Matrix<double, HorizonDim*(StateDim + InputDim), 1> b_qp =
        Eigen::MatrixXd::Zero(HorizonDim * (StateDim + InputDim), 1);

    // Q_bar_sparse_s. Setpoint gains.
    Eigen::SparseMatrix<double> Q_bar_sparse_s;

    // Q_bar_sparse_t. Trajectory gains.
    Eigen::SparseMatrix<double> Q_bar_sparse_t;

    // Phi_sparse  (depends on the model)
    Eigen::SparseMatrix<double> Phi_sparse;

    // C_bar_sparse
    Eigen::SparseMatrix<double> C_bar_sparse;

    // Gamma Sparse (depends on the model)
    Eigen::SparseMatrix<double> Gamma_sparse;

    // Soft Constraints Matrix (depends on the model)
    Eigen::SparseMatrix<double> G_sparse;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace controller
