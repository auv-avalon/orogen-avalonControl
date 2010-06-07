#ifndef PID_SETTINGS_UPDATE
#define PID_SETTINGS_UPDATE

static void setPIDSettings(PIDController& controller,
        motor_controller::PIDSettings const& settings)
{
    controller.setProportional(settings.p);
    controller.setIntegral(settings.i);
    controller.setDifferential(settings.d);
    controller.setMinMax(settings.min, settings.max);
}

static bool updatePIDSettings(PIDController& controller,
        motor_controller::PIDSettings& current, motor_controller::PIDSettings const& value)
{
    if (current != value)
    {
        current = value;
        setPIDSettings(controller, value);
        return true;
    }
    else
        return false;
}

#endif

