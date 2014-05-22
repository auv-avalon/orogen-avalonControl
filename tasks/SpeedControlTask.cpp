#include "SpeedControlTask.hpp"

//#include <avalonmath.h>

// Indentified values. Seee                                                                                                                                                        
//   AUV10/experiments/20100609_controller_calibration                                                                                                                             
// for the raw data                                                                                                                                                                
static const double SPEED_TO_PWM = 1.0 / 0.1122;                                                                                                                                   

using namespace avalon_control;

SpeedControlTask::SpeedControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : SpeedControlTaskBase(name, initial_state)
    , headingPID(new PIDController())
    , pitchPID(new PIDController())
{
    avalon_motor_controller::PIDSettings default_settings;
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
    pitchPID->reset();
    headingPID->reset();

    pitchPID->setSetpoint(0);
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

void SpeedControlTask::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    pitchPID->updatePIDSettings(current_pitch_pid,   _pitch_pid.get());
    headingPID->updatePIDSettings(current_heading_pid, _heading_pid.get());

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
    headingPID->setSetpoint(command.rotation_speed);
    pitchPID->setSetpoint(0);

    // Calculate all the commands for the motorboard
    Eigen::Vector3d euler_speeds = Avalonmath::toEuler(
            Avalonmath::vectorToAngleAxis(pose.angular_velocity));
    double delta_t = (pose.time - last_pose.time).toSeconds();

    double middle_vertical = SPEED_TO_PWM * command.linear_speeds[2];
    double rear_horizontal = headingPID->control(euler_speeds.x(), delta_t);
    double rear_vertical   = pitchPID->control(euler_speeds.y(), delta_t);

    double middle_horizontal = _y_factor.get() * pose.velocity.y();
    double left  = _x_factor.get() * pose.velocity.x();
    double right = left;
    last_pose = pose;

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
		hbridge::SimpleCommand hbridgeCommands;
		for(int i=0;i<HBRIGE_MAXIMUM_BOARDS;i++){
			hbridgeCommands.mode[i] = DM_PWM;
			hbridgeCommands.target[i] = 0;

		}

    // Apply correction factor from PWM-to-force response curve
    middle_vertical   = correct_pwm_value(middle_vertical, 0.14);
    middle_horizontal = correct_pwm_value(middle_horizontal, 0.14);
    rear_vertical     = correct_pwm_value(rear_vertical, 0.24);
    rear_horizontal   = correct_pwm_value(rear_horizontal, 0.14);
    left              = correct_pwm_value(left, 0.14);
    right             = correct_pwm_value(right, 0.14);

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
				hbridgeCommands.target[i] = values[i];
    }
    motor_commands.stamp = base::Time::now();
		hbridgeCommands.time = base::Time::now();
    _motor_commands.write(motor_commands);
		_hbridge_commands.write(hbridgeCommands);

    // Now export the internal controller states
    avalon_control::SpeedControllerState state;
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

