/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TrajectoryFollower.hpp"

using namespace avalon_control;

TrajectoryFollower::TrajectoryFollower(std::string const& name, TaskCore::TaskState initial_state)
    : TrajectoryFollowerBase(name, initial_state)
{
}

TrajectoryFollower::TrajectoryFollower(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TrajectoryFollowerBase(name, engine, initial_state)
{
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
    std::pair<double,double> p = spline.advance(spline.findOneClosestPoint(rbs.position, 0.1),_step_width.get(),0.1);
    std::pair<Eigen::Vector3d ,Eigen::Vector3d> p2 = spline.getPointAndTangent(p.first);
    Eigen::Vector3d next_point = p2.first;

    if(p.first == spline.getEndParam()){
        if(!endReached){
            endReachTime = rbs.time;
            endReached = true;
            state(ALIGN_AT_END);
        }
        if((rbs.time - endReachTime).toSeconds() >  _timeout_at_end_before_mark_end_reached.get()){
            state(REACHED_END);
        }
    }
    base::AUVPositionCommand cmd;
    cmd.x = next_point[0];
    cmd.y = next_point[1];
    cmd.z = next_point[2];
    Eigen::Vector3d direction = p2.second;
    direction.normalize();
    double heading = acos(direction[0]);
    if(direction[1] < 0) heading*=-1;
    cmd.heading = heading;
    _position_command.write(cmd);
    base::Waypoint wp(next_point,heading,0,0);
    printf("Heading: %f, from acos(%f)\n",heading,direction[0]);
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
