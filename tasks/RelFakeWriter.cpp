/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RelFakeWriter.hpp"

using namespace avalon_control;

RelFakeWriter::RelFakeWriter(std::string const& name, TaskCore::TaskState initial_state)
    : RelFakeWriterBase(name, initial_state)
{
}

RelFakeWriter::RelFakeWriter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : RelFakeWriterBase(name, engine, initial_state)
{
}

RelFakeWriter::~RelFakeWriter()
{
}

bool RelFakeWriter::setHeading(double value)
{
        lastCommand.heading = value;
	return(avalon_control::RelFakeWriterBase::setHeading(value));
}

bool RelFakeWriter::setX(double value)
{
        lastCommand.x = value;
	return(avalon_control::RelFakeWriterBase::setX(value));
}

bool RelFakeWriter::setY(double value)
{
        lastCommand.y = value;
	return(avalon_control::RelFakeWriterBase::setY(value));
}

bool RelFakeWriter::setZ(double value)
{
        lastCommand.z = value;
	return(avalon_control::RelFakeWriterBase::setZ(value));
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RelFakeWriter.hpp for more detailed
// documentation about them.

bool RelFakeWriter::configureHook()
{
    if (! RelFakeWriterBase::configureHook())
        return false;
    updateDynamicProperties();
    return true;
}
bool RelFakeWriter::startHook()
{
    if (! RelFakeWriterBase::startHook())
        return false;
    return true;
}
void RelFakeWriter::updateHook()
{
    RelFakeWriterBase::updateHook();
    _position_command.write(lastCommand); 
}
void RelFakeWriter::errorHook()
{
    RelFakeWriterBase::errorHook();
}
void RelFakeWriter::stopHook()
{
    RelFakeWriterBase::stopHook();
}
void RelFakeWriter::cleanupHook()
{
    RelFakeWriterBase::cleanupHook();
}
