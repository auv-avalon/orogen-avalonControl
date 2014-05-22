#ifndef AVALONCONTROL_PIDCONTROLLER_H
#define AVALONCONTROL_PIDCONTROLLER_H


//////////////////////////////////////////////////////////////////////
//THIS CONTROLLER IS DEPRECATED
//USE git://gitorious.org/rock-control/motor_controller.git INSTEAD 
//////////////////////////////////////////////////////////////////////


namespace avalon {

struct DOFSelection
{
    bool x;
    bool y;
    bool z;
    bool heading;
    DOFSelection()
        : x(0), y(0), z(0), heading(0){};
};

}

namespace avalon_motor_controller
{
    struct PIDSettings
    {
        double p;
        double i;
        double d;
        double min;
        double max;
#ifndef __orogen
        PIDSettings()
            : p(0), i(0), d(0), min(0), max(0) {}

        bool operator == (PIDSettings const& other) const
        { return p == other.p && i == other.i && d == other.d &&
            min == other.min && max == other.max; }
        bool operator != (PIDSettings const& other) const
        { return !(*this == other); }
#endif
    };

}


struct PIDControllerState {
        double integral_value;
        double diff;
        double settings[3];
        double terms[3]; // P, I and D parts in the value
        double delta_t; // delta time, in seconds, used for this update
	double result;

#ifndef __orogen
        PIDControllerState()
            : integral_value(0)
            , diff(0)
        {
            terms[0] = terms[1] = terms[2] = 0;
            settings[0] = settings[1] = settings[2] = 0;
        }
#endif
};

#ifndef __orogen
class PIDController {
public:
	PIDController(double proportional = 0.0,
                double integral     = 0.0,
                double differential = 0.0,
                double setpoint     = 0.0,
                double min          = 0.0,
                double max          = 0.0);

	~PIDController();

        void setMinMax(double min, double max);

	void setProportional(double prop);
	void setIntegral(double integ);
	void setDifferential(double diff);
	void reset();
	void setSetpoint(double setpoint);
	double getSetpoint();
	double controlByDiff(double diff, double delta_t);
	double control(double measurement, double delta_t);

	void setPIDSettings(avalon_motor_controller::PIDSettings const& settings);
	bool updatePIDSettings(avalon_motor_controller::PIDSettings& current, avalon_motor_controller::PIDSettings const& value);

        PIDControllerState getState() const;

private:
	double	proportional,
		integral,
		differential,
		setpoint;

        double min_value, max_value;
        PIDControllerState state;
};
#endif

#endif
