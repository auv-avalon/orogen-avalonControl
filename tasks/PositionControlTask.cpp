#include "PositionControlTask.hpp"
#include <base/commands/AUVMotion.hpp>

using namespace avalon_control;

PositionControlTask::PositionControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : PositionControlTaskBase(name, initial_state)
{
    xPID       = new PIDController();
    yPID       = new PIDController();

    avalon_motor_controller::PIDSettings default_settings;
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
    moving = true;
    return true;
}

void PositionControlTask::updateHook()
{
    
  /* 
   if(_pipeline_samples.connected()){
	controlData::Pipeline pipe;

	if(_pipeline_samples.read(pipe,true))
	{
	   if(pipe.inspection_state == controlData::ALIGN_AUV || pipe.inspection_state == controlData::FOLLOW_PIPE )
		moving = false;
	}
   }else
   {
	moving = true;
   }
   */


    xPID->updatePIDSettings(current_x_pid,   _x_pid.get());
    yPID->updatePIDSettings(current_y_pid,   _y_pid.get());
    
    base::samples::RigidBodyState pose_wrapper;
    if (_pose_samples.read(pose_wrapper) == RTT::NoData){
        state(WAITING_FOR_ORIENTATION);
        return;
    }

    base::samples::RigidBodyState pose(pose_wrapper);

    // Wait for at least two poses, in case we have an I part
    if (last_pose.time.isNull())
    {
        last_pose = pose;
        state(WAITING_FOR_ORIENTATION);
        return;
    }


    double time_step = (pose.time - last_pose.time).toSeconds();
    if (time_step == 0){
        state(WAITING_FOR_VALID_ORIENTATION);
        return;
    }

    //WE are valid here
    if(state() != RUNNING){
        state(RUNNING);
    }

    last_pose = pose;

    if (!_position_commands.read(last_command,false) == RTT::NewData)
    {
    	if (last_command_time.isNull()){
            state(WAITING_FOR_COMMAND);
	    return;
    	}if (_timeout.get() > 0.0 && (base::Time::now() - last_command_time).toSeconds() > _timeout.get())
	    return fatal();
    }
    else
    	last_command_time = base::Time::now();
    
    
    if(pose.cov_position(0,0) > _position_variance_threshold.get() || pose.cov_position(1,1) > _position_variance_threshold.get())
    {
      unstable_position = true;
      state(POSITION_VARIANCE_TO_HIGH);
    }
    else
    {
      unstable_position = false;
      state(RUNNING);
    }
 
   
   //Get Delta position (aka error)
   Eigen::Vector3d pos_delta = last_pose.position - Eigen::Vector3d(last_command.x,last_command.y,last_command.z);
   
   pos_delta[2]=0;

   //Transform current error in position to avalon thruster coordinate system to get the real mistion position in local coordinates
   Eigen::Vector3d local_delta =  pose.orientation.conjugate() * pos_delta;

    xPID->setSetpoint(0);//local_delta[0]);
    yPID->setSetpoint(0);//local_delta[1]);

    
    //double xSpeed= xPID->control(pose.position.x(), time_step);
    //double ySpeed= xPID->control(pose.position.y(), time_step);
    double xSpeed= xPID->control(local_delta[0], time_step);
    double ySpeed= yPID->control(local_delta[1], time_step);

    printf("Current Heading: %2.2f,%2.2f,%2.2f,%2.2f",pose.orientation.w(),pose.orientation.x(),pose.orientation.y(),pose.orientation.z());
    printf("Current Pos: %1.3f,%1.3f,%1.3f ",last_pose.position[0],last_pose.position[1],last_pose.position[2]);
    printf("Requested: %1.3f,%1.3f,%1.3f ",last_command.x,last_command.y,last_command.z);
    printf("Delta Values: %1.3f,%1.3f,%1.3f ",local_delta[0],local_delta[1],local_delta[2]);
    printf("Speed: %1.3f,%1.3f     \r",xSpeed,ySpeed);

    base::commands::AUVMotion motion_command;
    if(pos_delta.norm() < _optimal_heading_distance.get()){
        motion_command.heading = last_command.heading;
    } else {
        motion_command.heading = atan2(-pos_delta[1], -pos_delta[0]);
    }
    motion_command.z = last_command.z;//last_pose.position[2] - local_delta[2];// last_command.z;
    
    if(!unstable_position && moving)
    {
      motion_command.x_speed = xSpeed;
      motion_command.y_speed = ySpeed;
    }
    else
    {
      motion_command.x_speed = 0.0;
      motion_command.y_speed = 0.0;
    }
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

