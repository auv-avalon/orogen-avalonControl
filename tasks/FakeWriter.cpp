/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FakeWriter.hpp"

using namespace avalon_control;

FakeWriter::FakeWriter(std::string const& name, TaskCore::TaskState initial_state)
    : FakeWriterBase(name, initial_state)
{
}

FakeWriter::FakeWriter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : FakeWriterBase(name, engine, initial_state)
{
}

FakeWriter::~FakeWriter()
{
}

bool FakeWriter::setHeading(double value)
{
        lastCommand.heading = value;
  	//Call the base function, DO-NOT Remove
	return(avalon_control::FakeWriterBase::setHeading(value));
}

bool FakeWriter::setSpeed_x(double value)
{
        lastCommand.x_speed= value;

  	//Call the base function, DO-NOT Remove
	return(avalon_control::FakeWriterBase::setSpeed_x(value));
}

bool FakeWriter::setSpeed_y(double value)
{
        lastCommand.y_speed= value;

  	//Call the base function, DO-NOT Remove
	return(avalon_control::FakeWriterBase::setSpeed_y(value));
}

bool FakeWriter::setZ(double value)
{
        lastCommand.z = value;

  	//Call the base function, DO-NOT Remove
	return(avalon_control::FakeWriterBase::setZ(value));
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FakeWriter.hpp for more detailed
// documentation about them.

bool FakeWriter::configureHook()
{
    if (! FakeWriterBase::configureHook())
        return false;
    return true;
}
bool FakeWriter::startHook()
{
    if (! FakeWriterBase::startHook())
        return false;
    updateDynamicProperties();
    return true;
}
void FakeWriter::updateHook()
{
    FakeWriterBase::updateHook();
    _motion_commands.write(lastCommand); 
}

void FakeWriter::errorHook()
{
    FakeWriterBase::errorHook();
}
void FakeWriter::stopHook()
{
    FakeWriterBase::stopHook();
}
void FakeWriter::cleanupHook()
{
    FakeWriterBase::cleanupHook();
}
