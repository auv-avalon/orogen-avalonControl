#include "MotionControlTask.hpp"
#include <base/commands/Joints.hpp>
#include <cmath>
#include <base/commands/AUVMotion.hpp>

using namespace avalon_control;

static double constrain_angle(double value)
{
    if(!std::isfinite(value)){
        value = 0;
    }
    if (value < 0)
        return fmodf(value - M_PI, 2 * M_PI) + M_PI;
    else
        return fmodf(value + M_PI, 2 * M_PI) - M_PI;
}

MotionControlTask::MotionControlTask(std::string const& name, TaskCore::TaskState initial_state)
    : MotionControlTaskBase(name, initial_state)
{
    zPID       = new PIDController();
    headingPID = new PIDController();
    pitchPID   = new PIDController();

    avalon_motor_controller::PIDSettings default_settings;
    default_settings.min = -1;
    default_settings.max = 1;
    _z_pid.set(default_settings);
    _pitch_pid.set(default_settings);
    _heading_pid.set(default_settings);
    last_ground_position = -std::numeric_limits<double>::max();    
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
    dagon_mode = _dagon_mode.get() != 0;
    printf("Dagon mode is: %s",dagon_mode?"enabled":"disabled");
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

void MotionControlTask::updateHook()
{
    zPID->updatePIDSettings(current_z_pid,   _z_pid.get());
    headingPID->updatePIDSettings(current_heading_pid, _heading_pid.get());
    pitchPID->updatePIDSettings(current_pitch_pid, _pitch_pid.get());

    base::samples::RigidBodyState pose_wrapper;
    if (_pose_samples.readNewest(pose_wrapper) == RTT::NoData){
	return state(WAITING_FOR_ORIENTATION);
    }

    base::samples::RigidBodyState pose(pose_wrapper);

    // Wait for at least two poses, in case we have an I part
    if (last_pose.time.isNull())
    {
        last_pose = pose;
        return;
    }

    double time_step = (pose.time - last_pose.time).toSeconds();
    if (time_step == 0){
        return;
    }

    last_pose = pose;

    base::commands::AUVMotion new_command;
    if(_motion_commands.readNewest(new_command) == RTT::NewData)
    {
    	last_command = new_command;
    	last_command_time = base::Time::now();
    }
    else
    {
    	if (last_command_time.isNull()){
	    return state(WAITING_FOR_COMMAND);
	}

	if(_timeout.get() != 0){
	    	if ((base::Time::now() - last_command_time).toSeconds() > _timeout.get()){
		    return error(TIMEOUT);
		}
	}
    }
    last_command.heading = constrain_angle(last_command.heading);

    if(_use_min_ground_distance.get()){
	base::samples::RigidBodyState rbs;
	if(_ground_distance.connected()){
		while(_ground_distance.read(rbs) == RTT::NewData){
			if(rbs.position.z() != 0.0)
				last_ground_position = pose.position.z() - rbs.position.z(); 	
		}
	}else{
		last_ground_position = -std::numeric_limits<double>::max();	
	}
		_estimated_ground_pos.write(last_ground_position);
    	if(last_command.z - _min_ground_distance.get() < last_ground_position){
		last_command.z = last_ground_position + _min_ground_distance.get();	
	}
    }	

    // Update the PID controllers with the actual commands
    zPID->setSetpoint(last_command.z);
    headingPID->setSetpoint(last_command.heading);
    pitchPID->setSetpoint(_pitch_target.get());

    // Now update the sensor readings. Wrap the heading at PI/-PI if needed, to
    // match the given command.
    double current_z = pose.position.z();
    double current_heading = base::getYaw(pose.orientation);
    double current_pitch = base::getPitch(pose.orientation);

    if (current_heading - last_command.heading > M_PI)
        current_heading -= 2*M_PI;
    else if (current_heading - last_command.heading < -M_PI)
        current_heading += 2*M_PI;

    double middle_vertical = zPID->control(current_z, time_step);
    double dive = middle_vertical;
    double rear_horizontal = headingPID->control(current_heading, time_step);
    double turn_rate = rear_horizontal;
    double rear_vertical   = pitchPID->control(current_pitch, time_step);
    double pitch = rear_vertical;
    double middle_horizontal = _y_factor.get() * last_command.y_speed; //pose.velocity.y();
    double left  = _x_factor.get() * last_command.x_speed;//pose.velocity.x();
    if (left < -1.0) left = -1.0;
    else if (left > 1.0) left = 1.0;
    double right = left;

    // Now take into account couplings
    rear_vertical   += middle_vertical   * _z_coupling_factor.get();
    rear_horizontal -= middle_horizontal * _y_coupling_factor.get();
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
    if(dagon_mode){
       right =  rear_horizontal + right;
       left =  rear_horizontal + left;
    }

    // And finally convert all the values into the motcon commands
    base::actuators::Command hbridgeCommands;
    hbridgeCommands.resize(6);
    for(int i=0;i<6;i++){
        hbridgeCommands.mode[i] = base::actuators::DM_PWM;
        hbridgeCommands.target[i] = 0;

    }

    // Apply correction factor from PWM-to-force response curve
    middle_vertical   = correct_pwm_value(middle_vertical, 0.14);
    middle_horizontal = correct_pwm_value(middle_horizontal, 0.14);
    rear_vertical     = correct_pwm_value(rear_vertical, 0.14);
    rear_horizontal   = correct_pwm_value(rear_horizontal, 0.14);
    left              = correct_pwm_value(left, 0.14);
    right             = correct_pwm_value(right, 0.14);
    std::vector<float> values(6);
    values[MIDDLE_VERTICAL]   = DIR_MIDDLE_VERTICAL   * middle_vertical;
    values[MIDDLE_HORIZONTAL] = DIR_MIDDLE_HORIZONTAL * middle_horizontal;
    values[REAR_VERTICAL]     = DIR_REAR_VERTICAL     * rear_vertical;
    values[REAR_HORIZONTAL]   = DIR_REAR_HORIZONTAL   * rear_horizontal;
    values[LEFT]              = -DIR_LEFT              * left;
    values[RIGHT]             = -DIR_RIGHT             * right;


    int cs = dagon_mode?5:6;
    if(_cutoff.value().size() != cs){
        std::cerr << "Cutoffsize is invalid: " << _cutoff.value().size() << std::endl;
        error(CUTOFF_VECTOR_INVALID);
        return;
    }
    
    for (int i = 0; i < 6; ++i)
    {
        if(!std::isfinite(values[i])){

            std::cerr << "Waning got nan value within controller chain" << i << " lastheading " << last_command.heading << std::endl;
            values[i] = 0;
        }
	hbridgeCommands.target[i] = values[i];

	//Cutoff
	if(values[i] > _cutoff.value()[i]){
		values[i] = _cutoff.value()[i];
	}else if(values[i] < -_cutoff.value()[i]){
	        values[i] = -_cutoff.value()[i];
        }
	hbridgeCommands.target[i] = values[i];
    }
    
    hbridgeCommands.time = base::Time::now();
    _hbridge_commands.write(hbridgeCommands);
        
    base::commands::Joints jointCommands;
   
    if(dagon_mode){
        values.clear();
        values.resize(5);
        values[0] = dive + pitch;
        values[1] = _x_factor.get() * last_command.x_speed - turn_rate +  last_command.y_speed * _turn_coupling_factor.get();
        values[2] = _x_factor.get() * last_command.x_speed + turn_rate - last_command.y_speed * _turn_coupling_factor.get() ;
        values[3] = dive-pitch;
        values[4] = turn_rate + last_command.y_speed * _y_factor.get();
    }

    jointCommands = base::commands::Joints::Raw(values);
    jointCommands.time = hbridgeCommands.time;
    
    if(_joint_names.get().size() == values.size()){
      jointCommands.names = _joint_names.get();
    }else{
      std::cerr << "Wrong number of joint names. " << values.size() << " names are needed and got "<< _joint_names.get().size()  << std::endl;
      error(JOINT_NAMES_INVALID);
    }
    
    _joint_commands.write(jointCommands);
    avalon_control::MotionControllerState state_;
    state_.z_pid       = zPID->getState();
    state_.heading_pid = headingPID->getState();
    state_.pitch_pid   = pitchPID->getState();
    _debug.write(state_);
    return state(RUNNING);
}


void MotionControlTask::errorHook()
{
    sendStopCommand();
}
void MotionControlTask::stopHook()
{
    sendStopCommand();
}



// void MotionControlTask::cleanupHook()
// {
// }


void MotionControlTask::sendStopCommand(){
    base::actuators::Command hbridgeCommands;
    int cnt = dagon_mode?5:6; 
    hbridgeCommands.resize(cnt);
    std::vector<float> values;
    for(int i=0;i<cnt;i++){
        hbridgeCommands.mode[i] = base::actuators::DM_PWM;
        hbridgeCommands.target[i] = 0;
        values.push_back(0);

    }
    hbridgeCommands.time = base::Time::now();
    _hbridge_commands.write(hbridgeCommands);
    
    base::commands::Joints jointCommands = base::commands::Joints::Raw(values);
    jointCommands.time = hbridgeCommands.time;
    
    if(_joint_names.get().size() == values.size()){
      jointCommands.names = _joint_names.get();
    }
    else{
      std::cerr << "Wrong number of joint names. " << values.size() << " names are needed." << std::endl;
    }
    
    _joint_commands.write(jointCommands);
}
