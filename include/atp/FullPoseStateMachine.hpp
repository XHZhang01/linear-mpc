#pragma once

#include <atp/Helper.hpp>
#include <boost/sml.hpp>
#include <mpc/Helper.hpp>

namespace autopilot {
namespace full_pose_state_machine {

namespace sml = boost::sml;

// Events

/**
 * @brief      Triggered when an odometry message is received
 */
struct odometryMessageEvent {
    controller::MavState mavState;
};

/**
 * @brief      Triggered when a new reference command is added in the autopilot
 * queue
 */
struct newReferenceEvent {
};

/**
 * @brief      Triggered when the MPC queue size has to be checked
 */
struct checkMPCQueueEvent {
};

/**
 * @brief      Triggered when the Autopilot and MPC queue should be flushed
 */
struct FlushQueueEvent {
};

/**
 * @brief      Triggered when the Full pose state machine has to be terminated
 */
struct terminateFPEvent {
};

// SML events
static auto SML_odometryMessageEvent = sml::event<odometryMessageEvent>;
static auto SML_newReferenceEvent = sml::event<newReferenceEvent>;
static auto SML_checkMPCQueueEvent = sml::event<checkMPCQueueEvent>;
static auto SML_FlushQueueEvent = sml::event<FlushQueueEvent>;

static auto SML_terminateFPEvent = sml::event<terminateFPEvent>;

// States
class idle;       ///< Autopilot reference queue is empty
class waypoint;   ///< Active reference is a waypoint (position and orientation
                  ///< only)
class trajectory; ///< Active reference is a trajectory (position, orientation
                  ///< and their respective velocities)

// SML States
static auto SML_idleState = sml::state<idle>;
static auto SML_waypointState = sml::state<waypoint>;
static auto SML_trajectoryState = sml::state<trajectory>;

struct fullPoseControlStateMachine {
    auto operator()() const noexcept
    {
        // Guards

        // Switch from idle to waypoint control
        auto switchIdle2WaypointControl = [](std::shared_ptr<AutopilotQueue>& p_queueATP) {
            // Check if p_queueATP front == waypoint and return true;
            if (!p_queueATP->empty()) {
                return (p_queueATP->front().type() == typeid(Waypoint));
            }
            else {
                return false;
            }
        };

        // Switch from idle to trajectory control
        auto switchIdle2TrajectoryControl = [](std::shared_ptr<AutopilotQueue>& p_queueATP) {
            if (!p_queueATP->empty()) {
                return (p_queueATP->front().type() == typeid(Trajectory));
            }
            else {
                return false;
            }
        };

        // Checks whether the current waypoint has been reached by the MAV
        auto waypointReached = [](const odometryMessageEvent& p_event,
                                  std::shared_ptr<AutopilotQueue>& p_queueATP) {
            // Get the waypoint from the queue and check position tolerance and
            // orientation tolerance.
            const Waypoint activeWaypoint = boost::get<Waypoint>(p_queueATP->front());

            // Compute position error
            const double positionError =
                (activeWaypoint.position - p_event.mavState.position).norm();

            // Compute Quaternion error and angle error
            auto referenceQuaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            controller::euler2quat(0.0, 0.0, activeWaypoint.yaw, referenceQuaternion);

            const Eigen::Quaterniond orientationError =
                p_event.mavState.orientation.inverse().normalized()
                * referenceQuaternion.normalized();
            const double angleError = controller::constrainAngle(
                2.0 * std::acos(std::max(-1.0, std::min(1.0, std::fabs(orientationError.w())))));

            if ((positionError <= activeWaypoint.positionTolerance)
                && (angleError <= activeWaypoint.orientationTolerance)) {
                return true;
            }

            return false;
        };

        // Checks whether the reference trajectory has been executed by the MAV
        auto trajectoryExecuted = [](std::shared_ptr<controller::MPCQueue>& p_queueMPC) {
            // Check size of MPC Queue if empty return true
            return p_queueMPC->empty();
        };

        // True when the Autopilot queue is empty
        auto autopilotQueueIsEmpty = [](std::shared_ptr<AutopilotQueue>& p_queueATP) {
            return p_queueATP->empty();
        };

        // Actions

        // Pop the reference at the front of the Autopilot reference queue
        auto popReference = [](std::shared_ptr<AutopilotQueue>& p_queueATP) {
            if (!p_queueATP->empty()) {
                p_queueATP->pop_front();
            }
            else {
                // Todo -> Something is wrong, return to a safe state
            }
        };

        // Push the next reference at the MPC queue
        auto pushNextReference = [](std::shared_ptr<AutopilotQueue>& p_queueATP,
                                    std::shared_ptr<controller::MPCQueue>& p_queueMPC) {
            // Check if reference exists
            if (p_queueATP->empty()) {
                // Todo -> Something is wrong, return to a safe state
            }
            else {
                // Push the p_queueATP.front to the p_queueMPC
                if (p_queueATP->front().type() == typeid(Waypoint)) {
                    const Waypoint foo = boost::get<Waypoint>(p_queueATP->front());
                    // Push waypoint
                    p_queueMPC->push_back(ReferenceState(foo));
                }
                else if (p_queueATP->front().type() == typeid(Trajectory)) {
                    const Trajectory foo = boost::get<Trajectory>(p_queueATP->front());
                    // Push trajectory which is a sequence of reference setpoints
                    for (const auto referenceSetpoint : foo.trajectory) {
                        p_queueMPC->push_back(ReferenceState(referenceSetpoint));
                    }
                }
            }
        };

        // Clear Queues
        auto flushQueues = [](std::shared_ptr<AutopilotQueue>& p_queueATP,
                              std::shared_ptr<controller::MPCQueue>& p_queueMPC) {
            p_queueATP->clear();
            p_queueMPC->clear();
        };

        // StateMachine transition table
        return sml::make_transition_table(
            (*SML_idleState)
                + SML_newReferenceEvent[switchIdle2WaypointControl] / pushNextReference =
                SML_waypointState,
            SML_idleState
                + SML_newReferenceEvent[switchIdle2TrajectoryControl] / pushNextReference =
                SML_trajectoryState,
            SML_idleState[switchIdle2WaypointControl] / pushNextReference = SML_waypointState,
            SML_idleState[switchIdle2TrajectoryControl] / pushNextReference = SML_trajectoryState,
            SML_waypointState + SML_odometryMessageEvent[waypointReached] / popReference =
                SML_idleState,
            SML_waypointState + SML_odometryMessageEvent[autopilotQueueIsEmpty] =
                SML_idleState, ///> This transition can only happen when the
                               /// Autopilot Queue was flushed without triggering
                               /// the SML_FlushQueueEvent
            SML_waypointState + SML_FlushQueueEvent / flushQueues = SML_idleState,
            SML_trajectoryState + SML_checkMPCQueueEvent[trajectoryExecuted] / popReference =
                SML_idleState,
            SML_trajectoryState + SML_odometryMessageEvent[autopilotQueueIsEmpty] =
                SML_idleState, ///> This transition can only happen when the
                               /// Autopilot Queue was flushed without triggering
                               /// the SML_FlushQueueEvent
            SML_trajectoryState + SML_FlushQueueEvent / flushQueues = SML_idleState,
            SML_idleState + SML_terminateFPEvent = sml::X,
            SML_waypointState + SML_terminateFPEvent = sml::X,
            SML_trajectoryState + SML_terminateFPEvent = sml::X);
    }
};
} // namespace full_pose_state_machine
} // namespace autopilot
