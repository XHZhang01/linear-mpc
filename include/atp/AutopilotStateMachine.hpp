#pragma once

#include <atp/FullPoseStateMachine.hpp>
#include <atp/Helper.hpp>
#include <boost/sml.hpp>
#include <mpc/Helper.hpp>

namespace autopilot {
namespace autopilot_state_machine {

namespace sml = boost::sml;

// Events
struct ArmCommand {
};
struct DisarmCommand {
};
struct TakeOffCommand {
};

// SML events
static auto SML_ArmCommandEvent = sml::event<ArmCommand>;
static auto SML_DisarmCommandEvent = sml::event<DisarmCommand>;
static auto SML_TakeOffCommand = sml::event<TakeOffCommand>;

// States
class idle;            ///< MPC not activated - MAV can be controlled using pixhawk
                       ///< attitude controller
class fullPoseControl; ///< MPC activated.

// SML States
static auto SML_idleState = sml::state<idle>;
static auto SML_fullPoseControl =
    sml::state<full_pose_state_machine::fullPoseControlStateMachine>; ///< Composite
                                                                      ///< state

// Extra flags required for the state machine
struct AutopilotFlags {
    bool receivedImuMsg;
    bool receivedOdometryMsg;
    bool motorsArmed;

    AutopilotFlags(const bool p_receivedImuMsg = false,
                   const bool p_receivedOdometryMsg = false,
                   const bool p_motorsArmed = false) :
            receivedImuMsg(p_receivedImuMsg),
            receivedOdometryMsg(p_receivedOdometryMsg),
            motorsArmed(p_motorsArmed){};
};

struct autopilotStateMachine {
    auto operator()() const noexcept
    {
        // Guards

        // Switch to full pose control guard
        auto switch2FullPoseControlGuard = [](const AutopilotFlags& p_stateEstimationFlags) {
            return (p_stateEstimationFlags.receivedImuMsg
                    && p_stateEstimationFlags.receivedOdometryMsg);
        };

        // Actions
        auto move2FullPoseAction = []() {};

        // StateMachine transition table
        return sml::make_transition_table(
            SML_idleState + SML_ArmCommandEvent[switch2FullPoseControlGuard] = SML_fullPoseControl,
            SML_idleState + SML_TakeOffCommand[switch2FullPoseControlGuard] = SML_fullPoseControl,
            SML_idleState[switch2FullPoseControlGuard] / move2FullPoseAction = SML_fullPoseControl,
            (*SML_fullPoseControl) + SML_DisarmCommandEvent = SML_idleState);
        /*
    return sml::make_transition_table(
        (*SML_idleState) + SML_ArmCommandEvent[switch2FullPoseControlGuard] =
            SML_fullPoseControl,
        SML_idleState + SML_TakeOffCommand[switch2FullPoseControlGuard] =
            SML_fullPoseControl,
        SML_idleState[switch2FullPoseControlGuard] / move2FullPoseAction =
            SML_fullPoseControl,
        SML_fullPoseControl + SML_DisarmCommandEvent = SML_idleState);
    */
    }
};

// Some useful typedefs
typedef sml::sm<autopilotStateMachine> AutopilotStateMachine;

} // namespace autopilot_state_machine
} // namespace autopilot
