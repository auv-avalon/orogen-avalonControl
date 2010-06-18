#include "MotionControlTask.hpp"
#include <avalonmath.h>

#include "tasks/PIDSettingsUpdate.hpp"

using namespace avalon_control;

MotionControlTask::MotionControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : MotionControlTaskBase(name, initial_state)
{
    zPID       = new PIDController();
    headingPID = new PIDController();
    pitchPID   = new PIDController();

    motor_controller::PIDSettings default_settings;
    default_settings.min = -1;
    default_settings.max = 1;
    _z_pid.set(default_settings);
    _pitch_pid.set(default_settings);
    _heading_pid.set(default_settings);
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
    last_pose.invalidate();
    last_pose.time = base::Time();
    zPID->reset();
    headingPID->reset();
    pitchPID->reset();
    last_command_time = base::Time();
    return true;
}

static double correct_pwm_value(double value, double dead_zone)
{
    if (value > 0)
        return  dead_zone + (value * (1 - dead_zone));
    else if (value < 0)
        return -dead_zone + (value * (1 - dead_zone));
    else return value;
}

void MotionControlTask::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    updatePIDSettings(*zPID,   current_z_pid,   _z_pid.get());
    updatePIDSettings(*headingPID, current_heading_pid, _heading_pid.get());
    updatePIDSettings(*pitchPID, current_pitch_pid, _pitch_pid.get());

    wrappers::samples::RigidBodyState pose_wrapper;
    if (!_pose_samples.read(pose_wrapper))
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

    last_pose = pose;

    if (!_motion_commands.read(last_command))
    {
    	if (last_command_time.isNull())
	    return;
    	if ((base::Time::now() - last_command_time).toSeconds() > _timeout.get())
	    return fatal();
    }
    else
    	last_command_time = base::Time::now();

    if (fabs(last_command.heading) > M_PI)
        last_command.heading = fmod(last_command.heading + M_PI, 2 * M_PI) - M_PI;

//	printf("Target Heading: %f  X,Y,Z: %f,%f,%f\n",command.heading,command.x_speed,command.y_speed,command.z);
    // Update the PID controllers with the actual commands
    zPID->setSetpoint(last_command.z);
    headingPID->setSetpoint(last_command.heading);
    pitchPID->setSetpoint(0);

    // Now update the sensor readings. Wrap the heading at PI/-PI if needed, to
    // match the given command.
    double current_z = pose.position.z();
    double current_heading, current_pitch, current_roll;
    Avalonmath::quaternionToEuler(pose.orientation,
            current_heading, current_pitch, current_roll);

    if (current_heading - last_command.heading > M_PI)
        current_heading -= 2*M_PI;
    else if (current_heading - last_command.heading < -M_PI)
        current_heading += 2*M_PI;

    double middle_vertical = zPID->control(current_z, time_step);
    double rear_horizontal = headingPID->control(current_heading, time_step);
    double rear_vertical   = pitchPID->control(current_pitch, time_step);

    double middle_horizontal = _y_factor.get() * last_command.y_speed; //pose.velocity.y();
    double left  = _x_factor.get() * last_command.x_speed;//pose.velocity.x();
    if (left < -1.0) left = 1.0;
    else if (left > 1.0) left = 1.0;
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

    // Apply correction factor from PWM-to-force response curve
    middle_vertical   = correct_pwm_value(middle_vertical, 0.11);
    middle_horizontal = correct_pwm_value(middle_horizontal, 0.11);
    rear_vertical     = correct_pwm_value(rear_vertical, 0.15);
    rear_horizontal   = correct_pwm_value(rear_horizontal, 0.11);
    left              = correct_pwm_value(left, 0.11);
    right             = correct_pwm_value(right, 0.11);

    double values[6];
    values[MIDDLE_VERTICAL]   = DIR_MIDDLE_VERTICAL   * middle_vertical;
    values[MIDDLE_HORIZONTAL] = DIR_MIDDLE_HORIZONTAL * middle_horizontal;
    values[REAR_VERTICAL]     = DIR_REAR_VERTICAL     * rear_vertical;
    values[REAR_HORIZONTAL]   = DIR_REAR_HORIZONTAL   * rear_horizontal;
    values[LEFT]              = DIR_LEFT              * left;
    values[RIGHT]             = DIR_RIGHT             * right;

//	printf("Motor Values: ");
    for (int i = 0; i < 6; ++i)
    {
        motor_commands.isChanSet[i] = true;
        motor_commands.channels[i] = rint(values[i] * 200);
//	printf(" %f",values[i]);
    }
//    printf("\n");
    motor_commands.stamp = base::Time::now();
    _motor_commands.write(motor_commands);

    avalon_control::MotionControllerState state;
    state.z_pid       = zPID->getState();
    state.heading_pid = headingPID->getState();
    state.pitch_pid   = pitchPID->getState();
    _debug.write(state);
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

