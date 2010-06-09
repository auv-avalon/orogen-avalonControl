#include "SpeedControlTask.hpp"

#include "tasks/PIDSettingsUpdate.hpp"
#include <avalonmath.h>


using namespace avalon_control;

SpeedControlTask::SpeedControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : SpeedControlTaskBase(name, initial_state)
    , zPID(new PIDController())
    , headingPID(new PIDController())
    , pitchPID(new PIDController())
{
    motor_controller::PIDSettings default_settings;
    default_settings.min = -1;
    default_settings.max = 1;
    _z_pid.set(default_settings);
    _pitch_pid.set(default_settings);
    _heading_pid.set(default_settings);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SpeedControlTask.hpp for more detailed
// documentation about them.

// bool SpeedControlTask::configureHook()
// {
//     return true;
// }
bool SpeedControlTask::startHook()
{
    zPID->reset();
    pitchPID->reset();
    headingPID->reset();
    pitchPID->setSetpoint(0);
    return true;
}

void SpeedControlTask::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    updatePIDSettings(*zPID,       current_z_pid,       _z_pid.get());
    updatePIDSettings(*pitchPID,   current_pitch_pid,   _pitch_pid.get());
    updatePIDSettings(*headingPID, current_heading_pid, _heading_pid.get());

    avalon_control::SpeedCommand command;
    if (! _speed_commands.read(command))
        return; // no current command

    wrappers::samples::RigidBodyState pose_wrapper;
    if (!_pose_samples.read(pose_wrapper))
        return; // no pose reading
    base::samples::RigidBodyState pose(pose_wrapper);

    // Wait for at least two poses, in case we have an I part
    if (last_pose.time.isNull())
    {
        last_pose = pose;
        return;
    }

    // Update the PIDs with the set points
    zPID->setSetpoint(command.linear_speeds[2]);
    headingPID->setSetpoint(command.rotation_speed);
    pitchPID->setSetpoint(0);

    // Calculate all the commands for the motorboard
    Eigen::Vector3d euler_speeds = Avalonmath::toEuler(
            Avalonmath::vectorToAngleAxis(pose.angular_velocity));
    double delta_t = (pose.time - last_pose.time).toSeconds();

    double middle_vertical = zPID->control(pose.velocity.z(), delta_t);
    double rear_horizontal = headingPID->control(euler_speeds.x(), delta_t);
    double rear_vertical   = pitchPID->control(euler_speeds.y(), delta_t);

    double middle_horizontal = _y_factor.get() * pose.velocity.y();
    double left  = _x_factor.get() * pose.velocity.x();
    double right = left;

    // Now take into account couplings
    rear_vertical   += middle_vertical   * _z_coupling_factor.get();
    rear_horizontal += middle_horizontal * _y_coupling_factor.get();

    // Take into account the saturations due to . Namely, we prefer heading to
    // striving and pitch to depth. This is VERY crude.
    if (fabs(rear_vertical) > 1.0)
    {
        middle_vertical /= fabs(rear_vertical);
        rear_vertical   /= fabs(rear_vertical);
    }
    if (fabs(rear_horizontal) > 1.0)
    {
        middle_horizontal /= fabs(rear_horizontal);
        rear_horizontal   /= fabs(rear_horizontal);
    }

    // And finally convert all the values into the motcon commands
    controlData::Motcon motor_commands;

    double values[6];
    values[MIDDLE_VERTICAL]   = DIR_MIDDLE_VERTICAL   * middle_vertical;
    values[MIDDLE_HORIZONTAL] = DIR_MIDDLE_HORIZONTAL * middle_horizontal;
    values[REAR_VERTICAL]     = DIR_REAR_VERTICAL     * rear_vertical;
    values[REAR_HORIZONTAL]   = DIR_REAR_HORIZONTAL   * rear_horizontal;
    values[LEFT]              = DIR_LEFT              * left;
    values[RIGHT]             = DIR_RIGHT             * right;

    for (int i = 0; i < 6; ++i)
    {
        motor_commands.isChanSet[i] = true;
        motor_commands.channels[i] = rint(values[i] * 255);
    }
    motor_commands.stamp = base::Time::now();
    _motor_commands.write(motor_commands);

    // Now export the internal controller states
    avalon_control::SpeedControllerState state;
    state.z_pid = zPID->getState();
    state.heading_pid = headingPID->getState();
    state.pitch_pid = pitchPID->getState();
    _debug.write(state);
}

// void SpeedControlTask::errorHook()
// {
// }
// void SpeedControlTask::stopHook()
// {
// }
// void SpeedControlTask::cleanupHook()
// {
// }

