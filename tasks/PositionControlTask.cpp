#include "PositionControlTask.hpp"

#include <rtt/NonPeriodicActivity.hpp>

#include "tasks/PIDSettingsUpdate.hpp"


using namespace avalon_control;


RTT::NonPeriodicActivity* PositionControlTask::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }


PositionControlTask::PositionControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : PositionControlTaskBase(name, initial_state)
{
    xPID       = new PIDController();
    yPID       = new PIDController();

    motor_controller::PIDSettings default_settings;
    default_settings.min = -1;
    default_settings.max = 1;
    _x_pid.set(default_settings);
    _y_pid.set(default_settings);
}





/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PositionControlTask.hpp for more detailed
// documentation about them.

// bool PositionControlTask::configureHook()
// {
//     return true;
// }

bool PositionControlTask::startHook()
{
    last_pose.invalidate();
    last_pose.time = base::Time();
    xPID->reset();
    yPID->reset();
    last_command_time = base::Time();
    return true;
}

void PositionControlTask::updateHook(std::vector<RTT::PortInterface*> const& updated_ports)
{
    updatePIDSettings(*xPID,   current_x_pid,   _x_pid.get());
    updatePIDSettings(*yPID,   current_y_pid,   _y_pid.get());
    
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

    if (!_position_commands.read(last_command))
    {
    	if (last_command_time.isNull())
	    return;
    	if ((base::Time::now() - last_command_time).toSeconds() > _timeout.get())
	    return fatal();
    }
    else
    	last_command_time = base::Time::now();
 
   
   //Get Delta position (aka error)
   Eigen::Vector3d pos_delta = Eigen::Vector3d(last_command.x,last_command.y,last_command.z) - last_pose.position;

   //Transform current error in position to avalon thruster coordinate system to get the real mistion position in local coordinates
   Eigen::Vector3d local_delta = pose.orientation * pos_delta;

    xPID->setSetpoint(0);//local_delta[0]);
    yPID->setSetpoint(0);//local_delta[1]);

    
    //double xSpeed= xPID->control(pose.position.x(), time_step);
    //double ySpeed= xPID->control(pose.position.y(), time_step);
    double xSpeed= xPID->control(local_delta[0], time_step);
    double ySpeed= yPID->control(local_delta[1], time_step);


    base::AUVMotionCommand motion_command;
    motion_command.heading = last_command.heading;
    motion_command.z = last_pose.position[2] - local_delta[2];// last_command.z;
    motion_command.x_speed = xSpeed;
    motion_command.y_speed = ySpeed;
    _motion_commands.write(motion_command);


}

// void PositionControlTask::errorHook()
// {
// }
// void PositionControlTask::stopHook()
// {
// }
// void PositionControlTask::cleanupHook()
// {
// }
