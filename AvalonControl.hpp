#ifndef AVALON_CONTROL_HPP
#define AVALON_CONTROL_HPP

#include <pidcontroller.h>
#include <base/time.h>

namespace motor_controller
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

namespace avalon_control
{
    enum MOTCON_CHANNELS
    {
        MIDDLE_HORIZONTAL = 4,
        MIDDLE_VERTICAL   = 1,
        REAR_HORIZONTAL   = 0,
        REAR_VERTICAL     = 5,
        RIGHT = 2,
        LEFT  = 3
    };

    enum MOTCON_CHANNEL_DIRECTIONS
    {
        DIR_MIDDLE_HORIZONTAL = -1,
        DIR_MIDDLE_VERTICAL   = 1,
        DIR_REAR_HORIZONTAL   = -1,
        DIR_REAR_VERTICAL     = -1,
        DIR_RIGHT = -1,
        DIR_LEFT  = -1
    };

    struct MotionControllerState
    {
        PIDControllerState z_pid;
        PIDControllerState heading_pid;
    };

    struct SpeedControllerState
    {
        PIDControllerState z_pid;
        PIDControllerState heading_pid;
        PIDControllerState pitch_pid;
    };

    struct MotionCommand
    {
        base::Time time;
        double heading;    //! absolute heading, in radians (positive
                           //! counter-clockwise, has to be in -PI/PI)
        double z;          //! absolute altitude, in meters (goes positive upwards)
        double x_speed;    //! desired forward speed, in m/s
        double y_speed;    //! desired left speed, in m/s

#ifndef __orogen
        MotionCommand()
            : heading(0), z(0), x_speed(0), y_speed(0) {}
#endif
    };

    struct SpeedCommand
    {
        base::Time time;
        double linear_speeds[3]; //! x, y and z speeds in m/s
        double rotation_speed;   //! rotation around the yaw axis in radians/s

#ifndef __orogen
        SpeedCommand()
            : rotation_speed(0)
        {
            linear_speeds[0] = linear_speeds[1] = linear_speeds[2] = 0;
        }
#endif
    };
}

#endif

