#ifndef QPSOLVER_IMPL_HPP
#define QPSOLVER_IMPL_HPP

template<size_t NV, size_t NE>
QPSolver<NV, NE>::QPSolver() :
        hDimMem_(nullptr),
        hQpMem_(nullptr),
        hSolMem_(nullptr),
        hArgMem_(nullptr),
        hWorkspaceMem_(nullptr),
        H_(MatrixNV::Identity()),
        g_(VectorNV::Zero()),
        A_(MatrixNE::Zero()),
        b_(VectorNE::Zero()),
        uStar_(VectorNV::Zero()),
        Jb_(MatrixNV::Identity()),
        uMin_(VectorNV::Constant(-1e+5)),
        uMax_(VectorNV::Constant(1e+5)),
        lb_mask_(VectorNV::Ones()),
        ub_mask_(VectorNV::Ones()),
        idxb_(Eigen::Matrix<int, NV, 1>::LinSpaced(NV, 0, NV - 1)),
        C_(nullptr),
        d_lg_(nullptr),
        d_ug_(nullptr),
        Zl_(nullptr),
        Zu_(nullptr),
        zl_(nullptr),
        zu_(nullptr),
        idxs_(nullptr),
        d_ls_(nullptr),
        d_us_(nullptr)
{
    initialiseAndAllocate();
}

template<size_t NV, size_t NE>
QPSolver<NV, NE>::~QPSolver()
{
    freeBackendMemory();
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setCost(const MatrixNV& H, const VectorNV& g)
{
    // Set quadratic and linear term
    setQuadraticCostTerm(H);
    setLinearCostTerm(g);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setEqualityConstraints(const MatrixNE& A, const VectorNE& b)
{
    // Copy and set
    A_ = A;
    b_ = b;

    ::d_dense_qp_set_A(A_.data(), &hQp_);
    ::d_dense_qp_set_b(b_.data(), &hQp_);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setLinearCostTerm(const VectorNV& g)
{
    g_ = g;
    ::d_dense_qp_set_g(g_.data(), &hQp_);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setLowerBound(const VectorNV& uMin)
{
    uMin_ = uMin;
    ::d_dense_qp_set_lb(uMin_.data(), &hQp_);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setLowerBoundMask(const VectorNV& lb_mask)
{
    lb_mask_ = lb_mask;
    ::d_dense_qp_set_lb_mask(lb_mask_.data(), &hQp_);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setQuadraticCostTerm(const MatrixNV& H)
{
    // Copy and set
    H_ = H;
    ::d_dense_qp_set_H(H_.data(), &hQp_);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setUpperBound(const VectorNV& uMax)
{
    uMax_ = uMax;
    ::d_dense_qp_set_ub(uMax_.data(), &hQp_);
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::setUpperBoundMask(const VectorNV& ub_mask)
{
    ub_mask_ = ub_mask;
    ::d_dense_qp_set_ub_mask(ub_mask_.data(), &hQp_);
}

template<size_t NV, size_t NE>
bool QPSolver<NV, NE>::solve(VectorNV& soln)
{
    // Make sure the QP problem has been set
    if (!isInitialised)
        return false;

    // run Interior Point Method
    d_dense_qp_ipm_solve(&hQp_,
            &hSol_,
            &hArg_,
            &hWorkspace_); ///< ToDo -> also export the return status of the solver

    // Check the solver status
    if (!hWorkspace_.status) {
        // Solver converged
        d_dense_qp_sol_get_v(&hSol_, this->uStar_.data());

        // Copy over  the result and return
        soln = uStar_;

        return true;
    }
    else {
        // Solver failed with flag hWorkspace_.status
        return false;
    }

    return false;
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::initialiseAndAllocate()
{
    freeBackendMemory();
    // allocate the QP dimensions
    auto dim_size_ = d_dense_qp_dim_memsize();
    hDimMem_ = malloc(dim_size_);

    ::d_dense_qp_dim_create(&hDim_, hDimMem_);
    ::d_dense_qp_dim_set_all(NV, NE, NV, 0, 0, 0, &hDim_);

    // allocate the QP problem
    auto qp_size = ::d_dense_qp_memsize(&hDim_);
    hQpMem_ = malloc(qp_size);
    ::d_dense_qp_create(&hDim_, &hQp_, hQpMem_);

    ::d_dense_qp_set_all(H_.data(),
            g_.data(),
            A_.data(),
            b_.data(),
            idxb_.data(),
            uMin_.data(),
            uMax_.data(),
            C_,
            d_lg_,
            d_ug_,
            Zl_,
            Zu_,
            zl_,
            zu_,
            idxs_,
            d_ls_,
            d_us_,
            &hQp_);

    // Initialise the Box constraints masks
    d_dense_qp_set_lb_mask(lb_mask_.data(), &hQp_);
    d_dense_qp_set_ub_mask(ub_mask_.data(), &hQp_);

    // Alocate solution
    auto qp_sol_size = ::d_dense_qp_sol_memsize(&hDim_);
    hSolMem_ = malloc(qp_sol_size);
    ::d_dense_qp_sol_create(&hDim_, &hSol_, hSolMem_);

    // IPM arguments
    auto ipm_arg_size = ::d_dense_qp_ipm_arg_memsize(&hDim_);
    hArgMem_ = malloc(ipm_arg_size);
    ::d_dense_qp_ipm_arg_create(&hDim_, &hArg_, hArgMem_);

    // Set defaults
    ::d_dense_qp_ipm_arg_set_default(hpipm_mode::ROBUST, &hArg_);
    hArg_.iter_max = 150;
    hArg_.remove_lin_dep_eq = 1;

    // create workspace
    auto ipm_size = ::d_dense_qp_ipm_ws_memsize(&hDim_, &hArg_);
    hWorkspaceMem_ = malloc(ipm_size);
    ::d_dense_qp_ipm_ws_create(&hDim_, &hArg_, &hWorkspace_, hWorkspaceMem_);

    // Backend initialisation is complete.
    isInitialised = true;
}

template<size_t NV, size_t NE>
void QPSolver<NV, NE>::freeBackendMemory()
{
    free(hDimMem_);
    free(hQpMem_);
    free(hSolMem_);
    free(hArgMem_);
    free(hWorkspaceMem_);
    hDimMem_ = nullptr;
    hQpMem_ = nullptr;
    hSolMem_ = nullptr;
    hArgMem_ = nullptr;
    hWorkspaceMem_ = nullptr;
}

#endif
