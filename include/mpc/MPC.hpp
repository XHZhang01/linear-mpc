#pragma once

#include <model/FirstOrderModel.hpp>
#include <model/ThrustModel.hpp>
#include <mpc/Datatypes.hpp>
#include <qp/QPMatrices.hpp>
#include <qp/QPSolver.hpp>

namespace controller {

class MPC {
    public:

    MPC() = delete;

    /**
     * @brief      Destroys the object.
     */
    ~MPC()
    {
    }

    MPC(const FirstOrderModel<kHorizonLength>& p_model,
        const std::shared_ptr<MPCQueue>& p_mpcQueuePtr);

    /**
     * @brief      Computes the optimal control sequence and returns the first
     *             output to be applied to the system
     *
     * @param      command  The command to be sent to the drone
     *
     * @return     The True when the solver converged to a solution
     */
    bool computeControl(ControlCommands& command);

    /**
     * @brief      Returns the reference position.
     *
     * @return     The position reference.
     */
    const Eigen::Vector3d& getPositionReference(void) const;

    /**
     * @brief      Gets the queue size.
     *
     * @return     The queue size.
     */
    size_t getQueueSize() const;

    /**
     * @brief      Initializes the time-independent matrices used in the QP
     */
    void initialize();

    /**
     * @brief      Updates the orientation of the frame used for attitude control.
     *             The value is used to Transform the MPC commands to the correct
     *             frame used by the attitude controller.
     *
     * @param[in]  attitudeControllerFrameQuat  The orientation quaternion q_WA of
     * the frame used for attitude control.
     */
    void setAttitudeControlFrame(const Eigen::Quaterniond& attitudeControllerFrameQuat);

    /**
     * @brief      Sets the current MAV state (position, orientation and
     * velocities)
     *
     * @param[in]  The MAV state
     */
    void setMavState(const MavState& mavState);

    /**
     * @brief      Appends a discrete time reference trajectory to the controller
     * Queue.
     *
     * @param[in]  p_referenceTrajectory  The reference trajectory
     */
    void setReferenceTrajectory(const ReferenceTrajectory& p_referenceTrajectory);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /**
     * @brief      Computes (and if applicable passes to the QP solver) the
     * matrices that remain constant during the experiment. e.g. The input box
     * constraints, the state space model matrices etc.
     */
    void computeConstantMatrices();

    /**
     * @brief      Computes and passes to the QP solver the matrices that depend
     * on time varying quantities such as the reference and the initial state.
     */
    void computeTimeDependentMatrices();

    /**
     * @brief      Calculates the time varying state reference
     *
     * @param      S_y_bar  The s y bar
     */
    void computeStateReference(Eigen::Matrix<double, kStateDim * kHorizonLength, 1>& S_y_bar);

    /**
     * @brief      Converts a single State reference to an Eigen vector
     *
     * @param[in]  ref_state  The reference state
     * @param      s_y        The s y
     *
     */
    void referenseSetpoint2EigenVector(const ReferenseSetpoint& setpoint,
                                       Eigen::Matrix<double, kStateDim, 1>& s_y,
                                       const bool useVelocity = true,
                                       const bool useAcceleration = false) const;

    std::shared_ptr<MPCQueue> m_mpcQueuePtr;                       ///< Ptr to the MPC Queue
    QPMatrices<kStateDim, kInputDim, kHorizonLength> m_qpMatrices; ///< QP related matrices
    ReferenseSetpoint m_lastReference;               ///< Front element of the reference queue
                                                     ///< (Last given reference if queue empty)
    MavState mavState_;                              ///< Last MAV state
    Eigen::Quaterniond attitudeControllerFrameQuat_; ///< Pixhawk frame orientation
    ReferenceType m_referenceType;   ///< Type of reference message currently in the queue
    Eigen::Vector3d m_positionError; ///< Find alternative implementation for this

    bool m_hasReference; ///< Has the controller received any reference in the
                         ///< past ?

    QPSolver<kInputDim * kHorizonLength> solver_; ///< Backend solver for the QP problem

    FirstOrderModel<kHorizonLength> mavModel_; /// MAV model
};

} // namespace controller
