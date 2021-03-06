/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TrajectoryFollower.hpp"
#include <base/Logging.hpp>
#include <base/commands/AUVPosition.hpp>

using namespace avalon_control;

TrajectoryFollower::TrajectoryFollower(std::string const& name, TaskCore::TaskState initial_state)
    : TrajectoryFollowerBase(name, initial_state)
{
    _final_heading.set(base::unset<double>());
    _max_spline_jump_distance.set(base::unset<double>());
}

TrajectoryFollower::TrajectoryFollower(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TrajectoryFollowerBase(name, engine, initial_state)
{
    _final_heading.set(base::unset<double>());
    _max_spline_jump_distance.set(base::unset<double>());
}

TrajectoryFollower::~TrajectoryFollower()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TrajectoryFollower.hpp for more detailed
// documentation about them.

bool TrajectoryFollower::configureHook()
{
    if (! TrajectoryFollowerBase::configureHook())
        return false;
    return true;
}
bool TrajectoryFollower::startHook()
{
    if (! TrajectoryFollowerBase::startHook())
        return false;
    
    spline = _trajectory.get().spline;

    endReached= false;
    last_pos_on_spline=spline.getStartParam();
    return true;
}
void TrajectoryFollower::updateHook()
{
    TrajectoryFollowerBase::updateHook();
    base::samples::RigidBodyState rbs;
    if(_pose_samples.readNewest(rbs,true) == RTT::NoData){
        //state(WAITING_FOR_POSE);
        return;
    }
    
    try{

    double next_pos_on_spline = spline.findOneClosestPoint(rbs.position, _geometrical_resolution.get());
    _next_pos_on_spline.write(next_pos_on_spline); 
    _last_pos_on_spline.write(last_pos_on_spline);
    if(!base::isUnset<double>(_max_spline_jump_distance.get())){
        double dist = spline.length(last_pos_on_spline, next_pos_on_spline,_geometrical_resolution.get());
        _segment_dist.write(dist);
        if( _max_spline_jump_distance.get() < dist){
            //We are to far away from the spline, recovering to spline
            //Also possible if the spline is overlapping
            next_pos_on_spline = last_pos_on_spline;    
        }else{
        }
    }
    if(_deny_reverse.get()){
        if(next_pos_on_spline < last_pos_on_spline){
            next_pos_on_spline = last_pos_on_spline;
        }
    }
    last_pos_on_spline = next_pos_on_spline;
    std::pair<double,double> p = spline.advance(next_pos_on_spline,_step_width.get(),_geometrical_resolution.get());
    std::pair<Eigen::Vector3d ,Eigen::Vector3d> p2 = spline.getPointAndTangent(p.first);
   
    //Calculate the direction vector
    Eigen::Vector3d target_vector = p2.first - rbs.position; 
    //If speed is one, it equals p2.first
    Eigen::Vector3d next_point = rbs.position + (target_vector *  _trajectory.get().speed); 
    //Reset depth to target depth, depth control is handled perfectly, we want only to incluence the world x,y speed
    next_point[2] = p2.first[2];


    /*    
    printf("Direction %f,%f,%f \n",target_vector[0],target_vector[1],target_vector[2]);
    printf("Test %f,%f,%f \n",test[0],test[1],test[2]);
    printf("Second %f,%f,%f \n",next_point[0],next_point[1],next_point[2]);
    */

    base::commands::AUVPosition cmd;
    cmd.x = next_point[0];
    cmd.y = next_point[1];
    cmd.z = next_point[2];
    Eigen::Vector3d direction = p2.second;
    direction.normalize();
    double heading = acos(direction[0]);
    if(fabs(p.first - spline.getEndParam()) < 0.001){
        if(!endReached){
            endReachTime = rbs.time;
            endReached = true;
            state(ALIGN_AT_END);
        }
        if((rbs.time - endReachTime).toSeconds() >  _timeout_at_end_before_mark_end_reached.get()){
            state(REACHED_END);

            //Start spline from beginning, if required
            if(_loop_spline.get()){
                endReached= false;
                last_pos_on_spline=spline.getStartParam();
            }
        }
        if(!base::isUnset<double>(_final_heading.get())){
            heading = _final_heading.get();
        }
    }
    if(direction[1] < 0) heading*=-1;
    if(_use_zero_heading.get()){
        heading = 0;
    }
    cmd.heading = heading;
    _position_command.write(cmd);
    base::Waypoint wp(next_point,heading,0,0);

    if(state() == CANNOT_FIND_CLOSED_POINT){
        recover();
    }

    base::LinearAngular6DCommand world_cmd; 
    world_cmd.time = rbs.time;
    world_cmd.angular(0) = 0;
    world_cmd.angular(1) = 0;
    world_cmd.angular(2) = heading;
    
    world_cmd.linear(0) = next_point[0]; 
    world_cmd.linear(1) = next_point[1];
    world_cmd.linear(2) = next_point[2];
    _world_command.write(world_cmd);


    _next_position.write(wp);

    }catch(std::runtime_error e){
            LOG_ERROR_S << "got runtime error " << e.what() << std::endl;
            LOG_ERROR_S << "RBS position is: " << rbs.position[0] << " " << rbs.position[1] << " " << rbs.position[2] << std::endl;
            LOG_ERROR_S << "Spline length is: " << spline.length(spline.getStartParam(),spline.getEndParam(), _geometrical_resolution.get()) << std::endl;
            state(CANNOT_FIND_CLOSED_POINT);
    }

    
}
void TrajectoryFollower::errorHook()
{
    TrajectoryFollowerBase::errorHook();
}
void TrajectoryFollower::stopHook()
{
    TrajectoryFollowerBase::stopHook();
}
void TrajectoryFollower::cleanupHook()
{
    TrajectoryFollowerBase::cleanupHook();
}
