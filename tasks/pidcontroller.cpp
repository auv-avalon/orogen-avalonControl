//////////////////////////////////////////////////////////////////////
//THIS CONTROLLER IS DEPRECATED
//USE git://gitorious.org/rock-control/motor_controller.git INSTEAD 
//////////////////////////////////////////////////////////////////////

#include "pidcontroller.h"

PIDController::PIDController(	double proportional,
				double integral,
				double differential,
				double setpoint,
                                double min,
                                double max) 
	: proportional(proportional),
	  integral(integral),
	  differential(differential),
	  setpoint(setpoint),
          min_value(min), max_value(max)
{

}

PIDController::~PIDController() {
}

void PIDController::setProportional(double prop) {
	proportional = prop;
}

void PIDController::setIntegral(double integ) {
	integral = integ;
}

void PIDController::setDifferential(double diff) {
	differential = diff;
}

void PIDController::reset() {
        state = PIDControllerState();
}

void PIDController::setSetpoint(double setp) {
	setpoint = setp;
}

double PIDController::control(double measurement, double delta_t) {
	double e = setpoint - measurement;
	return controlByDiff(e, delta_t);
}

double PIDController::getSetpoint() {
	return setpoint;
}

void PIDController::setMinMax(double min, double max)
{
        min_value = min;
        max_value = max;
}

double PIDController::controlByDiff(double diff, double delta_t) {
        double integral_value = state.integral_value +
            integral * (diff + state.diff) / 2 * delta_t;

        state.settings[0] = proportional;
        state.settings[1] = integral;
        state.settings[2] = differential;
        state.terms[0] = proportional * diff;
        state.terms[1] = integral_value;
        state.terms[2] = differential * (diff - state.diff) / delta_t;
        state.diff     = diff;
        state.delta_t  = delta_t;

        double result = 
            (state.terms[0] + state.terms[1] + state.terms[2]);

        if (result > max_value)
            result = max_value;
        else if (result < min_value)
            result = min_value;
        else
	    state.integral_value = integral_value;
	state.result = result;
	return result;
}

PIDControllerState PIDController::getState() const
{
        return state;
}

bool PIDController::updatePIDSettings(avalon_motor_controller::PIDSettings& current, const avalon_motor_controller::PIDSettings& value)
{
    if (current != value)
    {
            current = value;
            setPIDSettings(value);
            return true;
    }
    else
            return false;
}

void PIDController::setPIDSettings(const avalon_motor_controller::PIDSettings& settings)
{
    setProportional(settings.p);
    setIntegral(settings.i);
    setDifferential(settings.d);
    setMinMax(settings.min, settings.max);
}


