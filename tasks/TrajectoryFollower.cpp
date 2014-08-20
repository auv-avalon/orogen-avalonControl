/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TrajectoryFollower.hpp"

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

    endReached= false;
    last_pos_on_spline=0;
    return true;
}
void TrajectoryFollower::updateHook()
{
    TrajectoryFollowerBase::updateHook();
    base::samples::RigidBodyState rbs;
    if(_pose_samples.read(rbs) == RTT::NoData){
        //state(WAITING_FOR_POSE);
        return;
    }
    
    base::geometry::Spline<3> spline = _trajectory.get().spline;
    double next_pos_on_spline = spline.findOneClosestPoint(rbs.position, _geometrical_resolution.get());

    if(!base::isUnset<double>(_max_spline_jump_distance.get())){
        if( _max_spline_jump_distance.get() < spline.length(last_pos_on_spline, next_pos_on_spline,_geometrical_resolution.get())){
            //We are to far away from the spline, recovering to spline
            //Also possible if the spline is overlapping
            next_pos_on_spline = last_pos_on_spline;    
        }
    }
    last_pos_on_spline = next_pos_on_spline;

    std::pair<double,double> p = spline.advance(spline.findOneClosestPoint(rbs.position, _geometrical_resolution.get()),_step_width.get(),_geometrical_resolution.get());
    std::pair<Eigen::Vector3d ,Eigen::Vector3d> p2 = spline.getPointAndTangent(p.first);
    Eigen::Vector3d next_point = p2.first;
    //printf("first: %f, end: %f\n",p.first,spline.getEndParam());
    base::AUVPositionCommand cmd;
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

    base::LinearAngular6DCommand world_cmd; 
    world_cmd.time = rbs.time;
    world_cmd.angular(0) = heading;
    world_cmd.angular(1) = 0;
    world_cmd.angular(2) = 0;
    
    world_cmd.linear(0) = next_point[0]; 
    world_cmd.linear(1) = next_point[1];
    world_cmd.linear(2) = next_point[2];
    _world_command.write(world_cmd);


    _next_position.write(wp);

    
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
