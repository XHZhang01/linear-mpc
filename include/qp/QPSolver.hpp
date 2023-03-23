#ifndef QPSOLVER_HPP
#define QPSOLVER_HPP

#include <Eigen/Core>

extern "C" {
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_timing.h>
}

/**
 * @brief      Class used for solving a QP common in linear MPC of the form:
 *
 * @tparam     NV    Number of variables to be optimised.
 * @tparam     NE    Number of equality constraints.
 */
template<size_t NV, size_t NE = NV>
class QPSolver {
    public:
    using MatrixNV = Eigen::Matrix<double, NV, NV>; ///< Square Matrix with NV, rows and cols.
    using VectorNV = Eigen::Matrix<double, NV, 1>;  ///< Vector with NV rows.

    using MatrixNE = Eigen::Matrix<double, NE, NV>; ///< Matrix with NE rows and NV cols.
    using VectorNE = Eigen::Matrix<double, NE, 1>;  ///< Vector with NE rows.

    /**
     * @brief      Constructs a new instance of the solver.
     */
    QPSolver();

    /**
     * @brief      Destroys the object.
     */
    ~QPSolver();

    /**
     * @brief      Sets the cost function matrices H and c
     *
     * @param[in]  H     The new value
     * @param[in]  g     The new value
     */
    void setCost(const MatrixNV& H, const VectorNV& g);

    /**
     * @brief      Sets the equality constraint matrices: Au = b;
     *
     * @param[in]  A     The new value of A.
     * @param[in]  b     The new value of b.
     */
    void setEqualityConstraints(const MatrixNE& A, const VectorNE& b);

    /**
     * @brief      Sets the linear cost term g used in the QP cost function:
     * (1/2)* u^T H u + g^T u
     *
     * @param[in]  g     The linear cost term g
     */
    void setLinearCostTerm(const VectorNV& g);

    /**
     * @brief      Sets the lower bound u_min used in the optimisation box
     * constraints u_min <= u <= u_max
     *
     * @param[in]  uMin  The lower bound of the optimisation variable.
     */
    void setLowerBound(const VectorNV& uMin);

    /**
     * @brief      Sets the lower bound box constraint mask. Use 0.0 to deactivate
     * a specific constraint and 1.0 to activate
     *
     * @param[in]  lb_mask  The lower bound box constraint mask
     */
    void setLowerBoundMask(const VectorNV& lb_mask);

    /**
     * @brief      Sets the quadratic cost term H used in the QP cost function:
     * (1/2)* u^T H u + g^T u
     *
     * @param[in]  H     The new value of H.
     */
    void setQuadraticCostTerm(const MatrixNV& H);

    /**
     * @brief      Sets the upper bound u_max used in the optimisationbox
     * constraints u_min <= u <= u_max
     *
     * @param[in]  uMax  The upper bound of the optimisation variable.
     */
    void setUpperBound(const VectorNV& uMax);

    /**
     * @brief      Sets the upper bound box constraint mask. Use 0.0 to deactivate
     * a constraint and 1.0 to activate.
     *
     * @param[in]  ub_mask  The ub mask
     */
    void setUpperBoundMask(const VectorNV& ub_mask);

    /**
     * @brief      Runs the QP problem using the latest data.
     *
     * @param      soln  The computed solution of the optimisation problem
     *
     * @return     True when the QP has been solved successfully. False otherwise.
     */
    bool solve(VectorNV& soln);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
     * @brief      Initialises the backend solver
     */
    void initialiseAndAllocate();

    /**
     *
     * @brief      Frees the memory used in the Backend solver
     */
    void freeBackendMemory();

    bool isInitialised; ///< Is the Backend solver ready ?

    // Backend solver variables
    void* hDimMem_;
    struct d_dense_qp_dim hDim_; ///< Problem dimensions

    void* hQpMem_;
    struct d_dense_qp hQp_; ///< Backend QP sovler

    void* hSolMem_;
    struct d_dense_qp_sol hSol_; ///< QP solution

    void* hArgMem_;
    struct d_dense_qp_ipm_arg hArg_; ///< IPM arguments

    void* hWorkspaceMem_;
    struct d_dense_qp_ipm_ws hWorkspace_; ///< statistics, solution status

    MatrixNV H_;     ///< Quadratic cost term H. (cost: 1/2 u^T H u + g^T u)
    VectorNV g_;     ///< Linear cost term g. (cost: 1/2 u^T H u + g^T u)
    MatrixNE A_;     ///< Eq. constraint matrix A. (equality constraints: A u = b)
    VectorNE b_;     ///< Eq. constraint vector b. (equality constraints: A u = b)
    VectorNV uStar_; ///< Solution of the Optimisation problem. u^*
    MatrixNV Jb_;    ///<
    VectorNV uMin_;  ///< Lower ineq. con u_{MIN}. (ineq. constraints: u_min <= u
                     ///< <=u_max)
    VectorNV uMax_;  ///< Upper ineq. con u_{MAX}. (ineq. constraints: u_min <= u
                     ///< <=u_max)

    VectorNV lb_mask_; ///< Masks used to deactivate (use 0.0) or activate (1.0) a
                       ///< specific lower bound constraint.
    VectorNV ub_mask_; ///< Masks used to deactivate (use 0.0) or activate (1.0) a
                       ///< specific lower bound constraint

    Eigen::Matrix<int, NV, 1> idxb_; //< Sparsity pattern for Box constraints.

    ///< Currently ignored parameters
    double* C_;
    double* d_lg_;
    double* d_ug_;
    double* Zl_;
    double* Zu_;
    double* zl_;
    double* zu_;
    int* idxs_;
    double* d_ls_;
    double* d_us_;
};

#include "QPSolver_impl.hpp"

#endif
