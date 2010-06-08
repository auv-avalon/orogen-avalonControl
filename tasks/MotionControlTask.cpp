#include "MotionControlTask.hpp"
#include <avalonmath.h>

#include "tasks/PIDSettingsUpdate.hpp"

using namespace avalon_control;

MotionControlTask::MotionControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : MotionControlTaskBase(name, initial_state)
{
    zPID = new PIDController();
    headingPID = new PIDController();
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MotionControlTask.hpp for more detailed
// documentation about them.

// bool MotionControlTask::configureHook()
// {
//     return true;
// }

bool MotionControlTask::startHook()
{
    zPID->reset();
    headingPID->reset();
    return true;
}

void MotionControlTask::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    updatePIDSettings(*zPID,   current_z_pid,   _z_pid.get());
    updatePIDSettings(*headingPID, current_heading_pid, _heading_pid.get());

    avalon_control::MotionCommand command;
    if (!_motion_commands.read(command))
        return;

    if (fabs(command.heading) > M_PI)
        command.heading = fmod(command.heading + M_PI, 2 * M_PI) - M_PI;

    wrappers::samples::RigidBodyState pose_wrapper;
    if (!_pose_readings.read(pose_wrapper))
        return;
    base::samples::RigidBodyState pose(pose_wrapper);

    // Wait for at least two poses, in case we have an I part
    if (last_pose.time.isNull())
    {
        last_pose = pose;
        return;
    }

    double time_step = (pose.time - last_pose.time).toSeconds();
    if (time_step == 0)
        return;

    // Update the PID controllers with the actual commands
    zPID->setSetpoint(command.z);
    headingPID->setSetpoint(command.heading);

    // Now update the sensor readings. Wrap the heading at PI/-PI if needed, to
    // match the given command.
    double current_z = pose.position.z();
    double current_heading, current_pitch, current_roll;
    Avalonmath::quaternionToEuler(pose.orientation,
            current_heading, current_pitch, current_roll);

    if (current_heading - command.heading > M_PI)
        current_heading -= 2*M_PI;
    else if (current_heading - command.heading < -M_PI)
        current_heading += 2*M_PI;

    double z_speed        = zPID->control(current_z, time_step);
    double rotation_speed = headingPID->control(current_heading, time_step);

    avalon_control::SpeedCommand speed_command;
    speed_command.time = base::Time::now();
    speed_command.linear_speeds[0] = command.x_speed;
    speed_command.linear_speeds[1] = command.y_speed;
    speed_command.linear_speeds[2] = z_speed;
    speed_command.rotation_speed = rotation_speed;
    _speed_commands.write(speed_command);


    avalon_control::MotionControllerState state;
    state.z_pid = zPID->getState();
    state.heading_pid = headingPID->getState();
    _debug.write(state);

    last_pose = pose;
}

// void MotionControlTask::errorHook()
// {
// }
// void MotionControlTask::stopHook()
// {
// }
// void MotionControlTask::cleanupHook()
// {
// }

