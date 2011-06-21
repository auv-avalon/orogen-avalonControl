/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MotionFeedbackTask.hpp"
#include "AvalonControl.hpp"

#include <vector>
#include <cassert>

using namespace avalon_control;

MotionFeedbackTask::MotionFeedbackTask(std::string const& name, TaskCore::TaskState initial_state)
    : MotionFeedbackTaskBase(name, initial_state)
{
}

MotionFeedbackTask::MotionFeedbackTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MotionFeedbackTaskBase(name, engine, initial_state)
{
}

MotionFeedbackTask::~MotionFeedbackTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MotionFeedbackTask.hpp for more detailed
// documentation about them.

// bool MotionFeedbackTask::configureHook()
// {
//     if (! MotionFeedbackTaskBase::configureHook())
//         return false;
//     return true;
// }
// bool MotionFeedbackTask::startHook()
// {
//     if (! MotionFeedbackTaskBase::startHook())
//         return false;
//     return true;
// }
void MotionFeedbackTask::updateHook()
{
    MotionFeedbackTaskBase::updateHook();

    // Pass over input directly to output
    while (_hbridge_feedback.read(motor_status) == RTT::NewData)
    {
        // re-order thrusters according to the model requirements.
        // see: avalonModel/Estimation.cpp -> setPWMLevels().
        ordered_motor_status.push_back(motor_status.states[LEFT]);                // model: O^1
        ordered_motor_status.push_back(motor_status.states[RIGHT]);               // model: O^2
        ordered_motor_status.push_back(motor_status.states[MIDDLE_HORIZONTAL]);   // model: O->3
        ordered_motor_status.push_back(motor_status.states[MIDDLE_VERTICAL]);     // model: Ov4
        ordered_motor_status.push_back(motor_status.states[REAR_HORIZONTAL]);     // model: O->5
        ordered_motor_status.push_back(motor_status.states[REAR_VERTICAL]);       // model: Ov6

        // replace old motor_statuses with ordered ones
        motor_status.states.assign(ordered_motor_status.begin(), ordered_motor_status.end());

        assert(motor_status.states.size() == ordered_motor_status.size());

        _hbridge_status.write(motor_status);
    }
}
// void MotionFeedbackTask::errorHook()
// {
//     MotionFeedbackTaskBase::errorHook();
// }
// void MotionFeedbackTask::stopHook()
// {
//     MotionFeedbackTaskBase::stopHook();
// }
// void MotionFeedbackTask::cleanupHook()
// {
//     MotionFeedbackTaskBase::cleanupHook();
// }

