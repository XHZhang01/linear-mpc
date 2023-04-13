#include <mpc/MPC.hpp>

namespace controller {
MPC::MPC(const FirstOrderModel<kHorizonLength>& p_model,
         const std::shared_ptr<MPCQueue>& p_mpcQueuePtr) :
        m_mpcQueuePtr(p_mpcQueuePtr),
        m_qpMatrices(),
        m_lastReference(),
        mavState_(),
        attitudeControllerFrameQuat_(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
        m_referenceType(ReferenceType::kSetpoint),
        m_positionError(Eigen::Vector3d::Zero()),
        m_hasReference(false),
        mavModel_(p_model)
{
    // Initialise all the matrices.
    initialize();
}

bool MPC::computeControl(ControlCommands& command)
{
    // There is no point in trying to compute control commands, If we have zero
    if (!m_hasReference)
        m_hasReference = !m_mpcQueuePtr->empty();

    if (!m_hasReference)
        return false;

    // QP matrices
    computeTimeDependentMatrices();

    // Solution along the time horizon.
    Eigen::Matrix<double, kHorizonLength * kInputDim, 1> uopt_ =
        Eigen::Matrix<double, kHorizonLength * kInputDim, 1>::Zero();

    // Check convergence and return the control commands
    if (solver_.solve(uopt_)) {
        // Get pitch from the solver
        double pitch_ref = uopt_(mavModel_.kUThetaIdx_);

        // Get roll from the solver
        double roll_ref = uopt_(mavModel_.kUPhiIdx_);

        // Get thrust from the solver
        double thrust_ref = uopt_(mavModel_.kUThrustIdx_);

        // Assemble command
        command.thrust = thrust_ref;

        // We need to transform the command to the frame used in pixhawk. {Wp}

        // World frame in PX4
        const Eigen::Quaterniond q_Wp_B = attitudeControllerFrameQuat_;
        const Eigen::Quaterniond q_Wp_W =
            (q_Wp_B * mavState_.orientation.inverse())
                .normalized(); // Rotation between the two World frames {W}
                               // from odometry and {Wp} from pixhawk
                               //
                               // Get the  Z_NBr axis {Br} -> reference Body
        Eigen::Quaterniond q_NB;
        euler2quat(roll_ref, pitch_ref, 0.0, q_NB);
        const Eigen::Vector3d z_NBr = q_NB.toRotationMatrix().col(2);

        // Transform to the World frame.
        Eigen::Matrix<double, 3, 3> C_WN = Eigen::Matrix<double, 3, 3>::Identity();
        const auto x_WN = mavState_.orientation.toRotationMatrix().col(0).normalized();
        C_WN.col(0) = x_WN;
        C_WN.col(1) = Eigen::Vector3d(0.0, 0.0, 1.0).cross(x_WN);

        // Reference B in World frame.
        const Eigen::Vector3d z_WBr = (C_WN * z_NBr).normalized();

        // Reference X in World frame.
        const Eigen::Vector3d x_WBr =
            Eigen::Quaterniond(
                std::cos(0.5 * m_lastReference.yaw), 0.0, 0.0, std::sin(0.5 * m_lastReference.yaw))
                .toRotationMatrix()
                .col(0);

        // Reference orientation defined in the World frame.
        Eigen::Matrix<double, 3, 3> C_WBr = Eigen::Matrix<double, 3, 3>::Identity();
        C_WBr.col(0) = x_WBr;
        C_WBr.col(1) = z_WBr.cross(x_WBr);
        C_WBr.col(2) = z_WBr;

        // Transform the reference to the pixhawk frame.
        command.orientation = q_Wp_W * Eigen::Quaterniond(C_WBr);
        command.timestamp = mavState_.timestamp;
    }
    else {
        // MPC did not converge. Send an open loop command
        command.thrust = -0.1; ///< Decelerate with -0.1 g
        command.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        command.timestamp = mavState_.timestamp;
        return false;
    }

    return true;
}

const Eigen::Vector3d& MPC::getPositionReference(void) const
{
    return m_lastReference.position;
}

size_t MPC::getQueueSize() const
{
    return m_mpcQueuePtr->size();
}

void MPC::initialize()
{
    // Check that the Gains are not zero.
    if (mavModel_.getTrackingStateGains().isZero(1e-3)) {
        std::cerr << "State gains all set to zero.\n";
        std::exit(EXIT_FAILURE);
    }

    // Check Trajectory gains
    if (mavModel_.getTrackingInputGains().isZero(1e-3)) {
        std::cerr << "Input gains all set to zero.\n";
        std::exit(EXIT_FAILURE);
    }

    // Check Constraints
    if (mavModel_.getBoxConstraintsMin().isZero(1e-3)
        || mavModel_.getBoxConstraintsMax().isZero(1e-3)) {
        std::cerr << "Box constraints all set to zero.\n";
        std::exit(EXIT_FAILURE);
    }

    // Convert Model from continuous time to discrete.
    if (!mavModel_.c2d()) {
        std::cerr << "Model discretisation failed. Check that the model is "
                     "initialised correctly\n";
        std::exit(EXIT_FAILURE);
    }

    // Compute the Matrices which remain constant at every iteration
    computeConstantMatrices();
}

void MPC::setAttitudeControlFrame(const Eigen::Quaterniond& attitudeControllerFrameQuat)
{
    attitudeControllerFrameQuat_ = attitudeControllerFrameQuat;
}

void MPC::setMavState(const MavState& mavState)
{
    mavState_ = mavState;
}

void MPC::setReferenceTrajectory(const ReferenceTrajectory& p_referenceTrajectory)
{
    if (!m_hasReference && !p_referenceTrajectory.empty())
        m_hasReference = true;

    // Trajectory is a sequence of setpoints, push all of them
    for (const auto& setpoint : p_referenceTrajectory) {
        m_mpcQueuePtr->push_back(setpoint);
    }
}


void MPC::computeConstantMatrices()
{
    // Compute Phi and save Phi_sparse
    m_qpMatrices.Phi_sparse = mavModel_.computePhi().sparseView();

    // Compute Gamma
    m_qpMatrices.Gamma_sparse = mavModel_.computeGamma().sparseView();

    // Control state gains.
    const Eigen::Matrix<double, kStateDim, kStateDim> Q = mavModel_.getTrackingStateGains();

    // Use consistent gains along the prediction horizon.
    // Q_bar = kron(In,Q);
    const Eigen::Matrix<double, kHorizonLength * kStateDim, kHorizonLength* kStateDim> Q_bar =
        Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, kHorizonLength>::Identity(),
                                Q);

    // Sparsify Q_bar_s Compute Q_bar_sparse_s
    m_qpMatrices.Q_bar_sparse_s = Q_bar.sparseView();

    // Trajectory gains. Copy the same gains as in the Setpoint case.
    m_qpMatrices.Q_bar_sparse_t = m_qpMatrices.Q_bar_sparse_s;

    // Compute C_bar = kron(In,Cd) and sparsify
    m_qpMatrices.C_bar = Eigen::kroneckerProduct(
        Eigen::Matrix<double, kHorizonLength, kHorizonLength>::Identity(), mavModel_.getCd());
    m_qpMatrices.C_bar_sparse = m_qpMatrices.C_bar.sparseView();

    // Compute R_bar_s = kron(In,Qu_s)
    m_qpMatrices.R_bar_s =
        Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, kHorizonLength>::Identity(),
                                mavModel_.getTrackingInputGains());

    // Compute R_bar_t = kron(In,Qu_t)
    m_qpMatrices.R_bar_t =
        Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, kHorizonLength>::Identity(),
                                mavModel_.getTrackingInputGains());

    // CVX matrices

    // Compute A_qp for the setpoint case
    Eigen::Matrix<double, kHorizonLength*(kStateDim + kInputDim), kHorizonLength* kInputDim>
        A_qp_s = Eigen::Matrix<double,
                               kHorizonLength*(kStateDim + kInputDim),
                               kHorizonLength * kInputDim>::Zero();

    // A_qp Upper part -> Q_bar * C_bar * m_qpMatrices.Gamma
    A_qp_s.block<kHorizonLength * kStateDim, kHorizonLength * kInputDim>(0, 0) =
        m_qpMatrices.Q_bar_sparse_s * m_qpMatrices.C_bar * m_qpMatrices.Gamma_sparse;

    // A_qp lower part-> R_bar
    A_qp_s.block<kHorizonLength * kInputDim, kHorizonLength * kInputDim>(kHorizonLength * kStateDim,
                                                                         0) = m_qpMatrices.R_bar_s;

    // Compute Q_s used in cvx solver
    m_qpMatrices.A_qp_s = A_qp_s;
    m_qpMatrices.Q_s = A_qp_s.transpose() * A_qp_s; ///< This is the quadratic Cost term

    // Compute A_qp for the trajectory case
    Eigen::Matrix<double, kHorizonLength*(kStateDim + kInputDim), kHorizonLength* kInputDim>
        A_qp_t = Eigen::Matrix<double,
                               kHorizonLength*(kStateDim + kInputDim),
                               kHorizonLength * kInputDim>::Zero();

    // A_qp Upper part -> Q_bar * C_bar * m_qpMatrices.Gamma
    A_qp_t.block<kHorizonLength * kStateDim, kHorizonLength * kInputDim>(0, 0) =
        m_qpMatrices.Q_bar_sparse_t * m_qpMatrices.C_bar * m_qpMatrices.Gamma_sparse;

    // A_qp lower part-> R_bar
    A_qp_t.block<kHorizonLength * kInputDim, kHorizonLength * kInputDim>(kHorizonLength * kStateDim,
                                                                         0) = m_qpMatrices.R_bar_t;

    // Compute Q_t used in cvx solver
    m_qpMatrices.A_qp_t = A_qp_t;
    m_qpMatrices.Q_t = A_qp_t.transpose() * A_qp_t; ///< This is the quadratic Cost term

    // Decide whether to pass Q_s or Q_t to the solver based on the current Active
    // reference
    if (m_referenceType == ReferenceType::kTrajectory) {
        solver_.setQuadraticCostTerm(2.0 * m_qpMatrices.Q_t);
    }
    else {
        solver_.setQuadraticCostTerm(2.0 * m_qpMatrices.Q_s);
    }

    // Initialize u_max
    const auto u_max_t = mavModel_.getBoxConstraintsMax();

    // Upper bound for all the stages
    m_qpMatrices.u_max =
        Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, 1>::Constant(1.0), u_max_t);

    // Pass u_max to the solver
    solver_.setUpperBound(m_qpMatrices.u_max);

    // Initialize u_min
    const auto u_min_t = mavModel_.getBoxConstraintsMin();

    // Lower bound for all the stages.
    m_qpMatrices.u_min =
        Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, 1>::Constant(1.0), u_min_t);

    // Pass u_min to the solver
    solver_.setLowerBound(m_qpMatrices.u_min);
}

void MPC::computeTimeDependentMatrices()
{
    /////////////////////////////////////////////////
    /// Compute S_y_bar, X_hat, S_u_bar
    /////////////////////////////////////////////////

    // S_y_bar -> time varying reference over the prediction horizon
    Eigen::Matrix<double, kStateDim * kHorizonLength, 1> S_y_bar =
        Eigen::Matrix<double, kStateDim * kHorizonLength, 1>::Zero();
    computeStateReference(S_y_bar);
    // std::cout << S_y_bar << std::endl;

    // S_u_bar -> time varying reference input
    Eigen::Matrix<double, kHorizonLength * kInputDim, 1> S_u_bar =
        Eigen::Matrix<double, kHorizonLength * kInputDim, 1>::Zero();

    // Compute X_hat and shift position error in S_y_bar to zero

    // Get MAV position in nav frame
    const Eigen::Vector3d rotatedMavPosition = world2nav(mavState_.orientation, mavState_.position);

    // Get Mav velocity in World frame
    Eigen::Vector3d linearVelocityWorld = Eigen::Vector3d::Zero();

    if (mavState_.linearVelocityFrame == ReferenceFrame::kBody) {
        body2world(mavState_.orientation, mavState_.linearVelocity, linearVelocityWorld);
    }
    else if (mavState_.linearVelocityFrame == ReferenceFrame::kWorld) {
        linearVelocityWorld = mavState_.linearVelocity;
    }
    else {
        // std::cout << "Unknown linear velocity frame ---> assuming linear velocity
        // in World frame" << std::endl;
        linearVelocityWorld = mavState_.linearVelocity;
    }

    // Get MAV velocity in nav frame
    const Eigen::Vector3d rotatedMavVelocity =
        world2nav(mavState_.orientation,
                  linearVelocityWorld); // TODO -> Check Again the conversion

    // Get Euler angle rates
    Eigen::Vector3d angularVelocityBody = Eigen::Vector3d::Zero();
    if (mavState_.angularVelocityFrame == ReferenceFrame::kBody) {
        angularVelocityBody = mavState_.angularVelocity;
    }
    else if (mavState_.angularVelocityFrame == ReferenceFrame::kWorld) {
        world2body(mavState_.orientation, mavState_.angularVelocity, angularVelocityBody);
    }
    else {
        // std::cout << "Unknown angular velocity frame ---> assuming angular rates
        // in World frame" << std::endl;
        world2body(mavState_.orientation, mavState_.angularVelocity, angularVelocityBody);
    }

    // Get Euler angles
    double roll, pitch, yaw;
    quat2euler(mavState_.orientation, roll, pitch, yaw);

    // MAV state at time 0 in nav frame
    Eigen::Matrix<double, kStateDim, 1> x_hat =
        mavModel_.buildStateVector(rotatedMavPosition, rotatedMavVelocity, pitch, roll);

    // Shift the position of the initial state to Zero and shift the references
    // accordingly. (engineering trick to avoid issues with numerics when the
    // position of the MAV and the references are in the order of kilometers)
    Eigen::Matrix<double, kStateDim, kStateDim> help_m =
        Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
    help_m(mavModel_.kXIdx_, mavModel_.kXIdx_) = -1.0;
    help_m(mavModel_.kYIdx_, mavModel_.kYIdx_) = -1.0;
    help_m(mavModel_.kZIdx_, mavModel_.kZIdx_) = -1.0;

    const Eigen::Matrix<double, kStateDim * kHorizonLength, 1> s_y_bar_norm =
        Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, 1>::Constant(1.0), help_m)
        * x_hat;
    S_y_bar += s_y_bar_norm;
    x_hat += help_m * x_hat;

    // Compute position error used in the integrator.
    m_positionError = Eigen::Vector3d(
        S_y_bar(mavModel_.kXIdx_), S_y_bar(mavModel_.kYIdx_), S_y_bar(mavModel_.kZIdx_));

    // Compute the C matrix. This is the linear cost term
    if (m_referenceType == ReferenceType::kTrajectory) {
        // Trajectory
        m_qpMatrices.b_qp.setZero();
        m_qpMatrices.b_qp.block<kHorizonLength * kStateDim, 1>(0, 0) =
            m_qpMatrices.Q_bar_sparse_t * S_y_bar
            - m_qpMatrices.Q_bar_sparse_t * m_qpMatrices.C_bar_sparse * m_qpMatrices.Phi_sparse
                * x_hat;
        m_qpMatrices.C = -2 * m_qpMatrices.b_qp.transpose() * m_qpMatrices.A_qp_t;

        // Pass to the QP solver
        solver_.setCost(m_qpMatrices.Q_t, m_qpMatrices.C);
    }
    else {
        // Setpoint
        m_qpMatrices.b_qp.setZero(); // TODO -> remove this when initialization is done properly
        m_qpMatrices.b_qp.block<kHorizonLength * kStateDim, 1>(0, 0) =
            m_qpMatrices.Q_bar_sparse_s * S_y_bar
            - m_qpMatrices.Q_bar_sparse_s * m_qpMatrices.C_bar_sparse * m_qpMatrices.Phi_sparse
                * x_hat;
        m_qpMatrices.C = -2.0 * m_qpMatrices.b_qp.transpose() * m_qpMatrices.A_qp_s;

        // Pass to the QP solver
        solver_.setCost(m_qpMatrices.Q_s, m_qpMatrices.C);
    }
}

void MPC::computeStateReference(Eigen::Matrix<double, kStateDim * kHorizonLength, 1>& s_y_bar)
{
    // Initialize reference type to trajectory mode
    m_referenceType = ReferenceType::kTrajectory;
    // If the MPC Queue is empty, push the last command as a setpoint.
    if (m_mpcQueuePtr->empty()) {
        // Push the last Set point command
        Eigen::Matrix<double, kStateDim, 1> s_y = Eigen::Matrix<double, kStateDim, 1>::Zero();

        // Rotate to N frame and copy over the results for all stages.
        referenseSetpoint2EigenVector(m_lastReference, s_y, false, false);

        // Reference is the same at every stage k of the MPC
        s_y_bar = Eigen::kroneckerProduct(Eigen::Matrix<double, kHorizonLength, 1>::Ones(), s_y);
        m_referenceType = ReferenceType::kSetpoint;
        return;
    }

    // Queue is not empty, we have to compute the stage wise references and append
    // them to s_y_bar
    size_t i = 0;
    ReferenseSetpoint referenceSetpointStage_i;
    while (i < kHorizonLength && i < m_mpcQueuePtr->size()) {
        // Get the reference from the Queue.
        referenceSetpointStage_i = m_mpcQueuePtr->at(i);

        // Rotate to N frame including velocity/acc data
        Eigen::Matrix<double, kStateDim, 1> s_y = Eigen::Matrix<double, kStateDim, 1>::Zero();
        referenseSetpoint2EigenVector(referenceSetpointStage_i, s_y, true, true);

        // Append to s_y_bar
        s_y_bar.block<kStateDim, 1>(i * kStateDim, 0) = s_y;

        // Save the element at the front of the Queue.
        if (i == 0) {
            m_lastReference = referenceSetpointStage_i;
        }

        // Prepare for next iteration
        i++;
    }

    // Pop one reference from the Queue
    m_mpcQueuePtr->pop_front();

    // Fill in the rest stage-wise references. Applicable when the Queue size is
    // less than the horizon.
    while (i < kHorizonLength) {
        // Use last reference with Zero velocities this time.
        Eigen::Matrix<double, kStateDim, 1> s_y = Eigen::Matrix<double, kStateDim, 1>::Zero();
        referenseSetpoint2EigenVector(referenceSetpointStage_i, s_y, false, false);
        s_y_bar.block<kStateDim, 1>(i * kStateDim, 0) = s_y;

        // Prepare for next iteration
        i++;
    }

    // Update reference type.
    m_referenceType = ReferenceType::kTrajectory;
    return;
}

void MPC::referenseSetpoint2EigenVector(const ReferenseSetpoint& setpoint,
                                        Eigen::Matrix<double, kStateDim, 1>& s_y,
                                        const bool useVelocity,
                                        const bool useAcceleration) const
{
    // The reference state is expressed in the World frame. The reference vector
    // s_y should be in {N} frame. First compute the q_WN quaternion which should
    // only contain rotation around z.

    // Compute the projection of the x_B axis
    const Eigen::Vector3d x_WB = mavState_.orientation.toRotationMatrix().col(0);
    Eigen::Vector3d x_WB_projected(x_WB(0), x_WB(1), 0.0);

    // Check for singularity.
    if (x_WB_projected.norm() < 1e-3) {
        // ToDo -> Think how to handle.
    }

    // Construct the rotation matrix C_WN
    const Eigen::Vector3d x_WN = x_WB_projected.normalized();
    const Eigen::Vector3d z_WN(0.0, 0.0, 1.0);
    const Eigen::Vector3d y_WN = z_WN.cross(x_WN);
    Eigen::Matrix3d C_WN = Eigen::Matrix3d::Identity();
    C_WN.col(0) = x_WN;
    C_WN.col(1) = y_WN;
    C_WN.col(2) = z_WN;

    const Eigen::Matrix3d C_NW = C_WN.transpose();

    // Rotate the references to the {N} frame. r_WN is always zero.
    // Position
    const Eigen::Vector3d r_NBr = C_NW * setpoint.position;

    // Linear velocity
    Eigen::Vector3d v_NBr = Eigen::Vector3d::Zero();
    if (useVelocity) {
        v_NBr = C_NW * setpoint.linearVelocity;
    }

    // Linear acceleration
    Eigen::Vector3d a_NBr = Eigen::Vector3d::Zero();
    if (useAcceleration) {
        a_NBr = C_NW * setpoint.linearAcceleration;
    }

    // Linear acceleration, should be transformed to roll and pitch based
    // on the model. // ToDo -> Augment model to support these.
    const double pitch_ff = 0.0;
    const double roll_ff = 0.0;

    /*
    s_y = (Eigen::Matrix<double, kStateDim, 1>() << r_NBr(0), v_NBr(0),
    pitch_ff, r_NBr(1), v_NBr(1), roll_ff, r_NBr(2), v_NBr(2)) .finished();
              */

    s_y = mavModel_.buildStateVector(r_NBr, v_NBr, pitch_ff, roll_ff);
}

} // namespace controller
