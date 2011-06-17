/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MotionFeedbackTask.hpp"

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

